"""BILBO autonomous person-tracking loop.

State machine architecture and state set are unchanged. What changed is the
bookkeeping around the transitions, which is where the defects were.

The loop runs at a fixed rate on its own clock. It is no longer paced by the
camera, and every tick is wrapped so that a Python exception cannot terminate
autonomous operation.
"""

import csv
import logging
import logging.handlers
import os
import signal
import sys
import time

import config as cfg
import camera
import controller
import drone
import led
import lidar
import tracker
from state import DroneState

_log = logging.getLogger("bilbo")

# RC enable switch positions.
OFF, YAW_ONLY, FULL = 0, 1, 2
_ENABLE_NAMES = {OFF: "OFF", YAW_ONLY: "YAW_ONLY", FULL: "FULL"}

# ------------------------------------------------------------------ state ----

state = DroneState.IDLE

prev_enable = None
engage_armed = False        # latched low->high edge, cleared when it goes low
confirm_count = 0           # consecutive frames with a target, in READY

lost_since = None           # when the current target gap started
reacquire_count = 0
last_error_x = 0.0
last_confident_track = None  # cumulative watchdog, not resettable by flip-flop
track_started = None

search_start = None
search_dir = 1.0            # +1 = yaw right, matching the corrected sign

last_dist = None
last_dist_t = None
jump_count = 0

vision_faults = 0

self_commanded_mode = None  # a mode WE requested, vs the pilot changing mode
rtl_requested = False
emergency_since = None
rtl_attempts = 0
land_attempted = False

fault_latch = None
consecutive_faults = 0
_slow_tick = False          # set when a blocking mode change is expected
_shutdown = False

_csv_file = None
_csv_writer = None
_last_csv_flush = 0.0
_last_status_log = 0.0


# ---------------------------------------------------------------- logging ----

def _setup_logging():
    root = logging.getLogger()
    root.setLevel(getattr(logging, cfg.LOG_LEVEL, logging.INFO))
    fmt = logging.Formatter(
        "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
    )

    try:
        os.makedirs(cfg.LOG_DIR, exist_ok=True)
        handler = logging.handlers.RotatingFileHandler(
            os.path.join(cfg.LOG_DIR, "bilbo.log"),
            maxBytes=cfg.LOG_MAX_BYTES,
            backupCount=cfg.LOG_BACKUPS,
        )
        handler.setFormatter(fmt)
        root.addHandler(handler)
    except Exception:
        # Fall back to stderr rather than dying because a log path is bad.
        stream = logging.StreamHandler(sys.stderr)
        stream.setFormatter(fmt)
        root.addHandler(stream)
        root.error("file logging unavailable; using stderr", exc_info=True)
        return

    if cfg.DEBUG_CONSOLE:
        # Never on by default. print()/stdout at loop rate is blocking I/O: if
        # the consumer stops draining the pipe, the write blocks and the flight
        # loop freezes.
        stream = logging.StreamHandler(sys.stderr)
        stream.setFormatter(fmt)
        root.addHandler(stream


CSV_COLUMNS = [
    "t", "state", "enable", "mode", "armed", "alt_m", "link_age_s",
    "rc_age_s", "vision_age_s", "n_persons", "error_x", "error_y",
    "bearing_deg", "dist_cm", "dist_age_s", "yaw_rate", "forward",
    "gate", "tick_s", "fault",
]


def _open_csv():
    """Full-rate flight record. Without this the first anomaly is
    undiagnosable, and there will be one."""
    global _csv_file, _csv_writer
    try:
        path = os.path.join(cfg.LOG_DIR, "flight-%d.csv" % int(time.time()))
        _csv_file = open(path, "w", newline="", buffering=1024 * 64)
        _csv_writer = csv.writer(_csv_file)
        _csv_writer.writerow(CSV_COLUMNS)
        _log.info("flight log: %s", path)
    except Exception:
        _log.exception("could not open CSV flight log")
        _csv_file = None
        _csv_writer = None


def _fmt(value, spec):
    return "" if value is None else spec % value


def _write_record(record, now):
    """Append one row for this tick. Called for every tick without exception."""
    global _last_csv_flush
    if _csv_writer is None:
        return
    try:
        _csv_writer.writerow([
            "%.3f" % now,
            record.get("state", ""),
            record.get("enable", ""),
            record.get("mode") or "",
            int(bool(record.get("armed"))),
            _fmt(record.get("alt"), "%.2f"),
            "%.2f" % drone.link_age(),
            "%.2f" % drone.rc_age(),
            "%.3f" % record.get("vision_age", float("inf")),
            _fmt(record.get("n_persons"), "%d"),
            "%.1f" % record.get("error_x", 0.0),
            "%.1f" % record.get("error_y", 0.0),
            "%.2f" % record.get("bearing_deg", 0.0),
            _fmt(record.get("distance_cm"), "%d"),
            _fmt(None if last_dist_t is None else distance_age(now), "%.2f"),
            "%.2f" % record.get("yaw_rate", 0.0),
            "%.3f" % record.get("forward", 0.0),
            record.get("gate", ""),
            "%.4f" % record.get("tick_s", 0.0),
            fault_latch or "",
        ])
        if now - _last_csv_flush >= cfg.CSV_FLUSH_INTERVAL_S:
            _csv_file.flush()
            _last_csv_flush = now
    except Exception:
        _log.exception("CSV write failed")


def _status(now, message, *args):
    """Throttled human-readable line. The CSV carries the full-rate data."""
    global _last_status_log
    if now - _last_status_log < cfg.STATUS_LOG_INTERVAL_S:
        return
    _last_status_log = now
    _log.info(message, *args)


# --------------------------------------------------------------- shutdown ----

def _on_signal(signum, _frame):
    global _shutdown
    _shutdown = True
    _log.warning("signal %d received; shutting down", signum)


# ------------------------------------------------------------- soft reset ----

def soft_reset(keep_target=False):
    """Clear every latched value so re-engagement starts clean.

    Without this, a wound-up PID, a stale retained distance and a stale tracker
    target all carried across an engagement boundary.

    keep_target preserves the tracker's current identity. Used on engagement,
    where READY has just spent TARGET_CONFIRM_FRAMES establishing which person
    we are following -- discarding that would force a re-designation by largest
    box, which is the behaviour target persistence exists to avoid.
    """
    global lost_since, reacquire_count, confirm_count, last_error_x
    global last_dist, last_dist_t, jump_count, vision_faults, track_started

    controller.reset()
    if not keep_target:
        tracker.reset()

    lost_since = None
    reacquire_count = 0
    confirm_count = 0
    last_error_x = 0.0
    last_dist = None
    last_dist_t = None
    jump_count = 0
    vision_faults = 0
    track_started = None


# ---------------------------------------------------------------- sensors ----

def update_distance(new_cm, now):
    """Accept a new range only if it is plausible.

    A single large step is rejected; it must repeat before it is believed. The
    ~2 deg beam slips past a person onto the ground or sky behind them, and
    without this a one-frame slip commands full forward or full reverse.
    """
    global last_dist, last_dist_t, jump_count

    if new_cm is None:
        return

    if (last_dist is not None
            and abs(new_cm - last_dist) > cfg.LIDAR_MAX_JUMP_CM):
        jump_count += 1
        if jump_count < cfg.LIDAR_JUMP_CONFIRMS:
            _log.debug("rejecting range jump %d -> %d cm", last_dist, new_cm)
            return

    jump_count = 0
    last_dist = new_cm
    last_dist_t = now


def distance_or_none(now):
    """The retained range, or None if it is too old to act on."""
    if last_dist is None or last_dist_t is None:
        return None
    if now - last_dist_t > cfg.LIDAR_MAX_AGE_S:
        return None
    return last_dist


def distance_age(now):
    if last_dist_t is None:
        return float("inf")
    return now - last_dist_t


def read_enable(now):
    """Decode the 3-position AI switch. Stale or missing RC reads as OFF."""
    raw = drone.get_channel(cfg.CH_ENABLE)
    if raw is None or drone.rc_age() > cfg.RC_STALE_S:
        return OFF
    if raw < cfg.SWITCH_MID_LOW_US:
        return OFF
    if raw < cfg.SWITCH_MID_HIGH_US:
        return YAW_ONLY
    return FULL


# ------------------------------------------------------------ transitions ----

def request_mode(mode, timeout=1.5, retries=1):
    """Ask the Pixhawk for a mode and record that WE asked.

    Short timeout and low retry count so a mode change fits inside a tick
    budget; _slow_tick tells the outer loop not to treat the overrun as a stall.
    """
    global _slow_tick, self_commanded_mode
    _slow_tick = True
    if drone.set_mode(mode, timeout=timeout, retries=retries):
        self_commanded_mode = mode
        return True
    return False


def enter_rtl(reason):
    global state, rtl_requested
    if state == DroneState.RTL:
        return
    _log.warning("-> RTL (%s)", reason)
    state = DroneState.RTL
    rtl_requested = False


def enter_emergency(reason):
    global state, emergency_since, rtl_attempts, land_attempted
    if state == DroneState.EMERGENCY:
        return
    _log.error("-> EMERGENCY (%s)", reason)
    state = DroneState.EMERGENCY
    emergency_since = None
    rtl_attempts = 0
    land_attempted = False


# --------------------------------------------------------------- preflight ----

def _wait_for(predicate, timeout, interval=0.05, pump=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pump is not None:
            pump()
        try:
            if predicate():
                return True
        except Exception:
            _log.exception("preflight predicate raised")
        time.sleep(interval)
    return False


def preflight():
    """Prove every sensor the flight loop depends on before signalling READY.

    Ordering matters: the flight controller comes first because it is the
    component whose absence must abort. The original started the camera first
    and then blocked forever in wait_heartbeat().

    Battery, GPS, compass and EKF checks are deliberately absent -- those are
    the Pixhawk's responsibility.
    """
    led.init()
    led.selftest()

    if not drone.connect(timeout=10.0):
        _log.error("PREFLIGHT FAIL: no MAVLink heartbeat")
        return False

    try:
        lidar.init()
    except Exception:
        _log.exception("PREFLIGHT FAIL: LiDAR could not be opened")
        return False

    valid = {"n": 0}

    def lidar_healthy():
        if lidar.read_data() is not None:
            valid["n"] += 1
        return valid["n"] >= cfg.LIDAR_HEALTH_FRAMES

    if not _wait_for(lidar_healthy, timeout=3.0):
        _log.error("PREFLIGHT FAIL: LiDAR gave %d/%d valid frames (%s)",
                   valid["n"], cfg.LIDAR_HEALTH_FRAMES, lidar.health())
        return False

    try:
        width, height = camera.intitalise()
    except Exception:
        _log.exception("PREFLIGHT FAIL: camera did not start")
        return False

    if not _wait_for(camera.inference_alive, timeout=8.0):
        _log.error("PREFLIGHT FAIL: no inference output from the IMX500")
        return False

    try:
        tracker.configure(width, height)
    except Exception:
        _log.exception("PREFLIGHT FAIL: camera geometry mismatch")
        return False

    if not _wait_for(
        lambda: drone.get_channel(cfg.CH_ENABLE) is not None,
        timeout=5.0,
        pump=drone.poll,
    ):
        _log.error("PREFLIGHT FAIL: no RC_CHANNELS stream -- ch%d unreadable, "
                   "so autonomy could never be enabled", cfg.CH_ENABLE)
        return False

    _log.info("preflight OK (lidar %s, camera %s)",
              lidar.health(), camera.health())
    return True


# --------------------------------------------------------------- the loop ----

def run_tick(now, tick_duration):
    global state, prev_enable, engage_armed, confirm_count
    global lost_since, reacquire_count, last_error_x, last_confident_track
    global track_started, vision_faults, fault_latch
    global self_commanded_mode, rtl_requested
    global emergency_since, rtl_attempts, land_attempted
    global search_start, search_dir

    drone.poll()
    drone.send_heartbeat()

    # Every return path returns this record, so no tick escapes the flight log.
    # The early returns are the most diagnostically valuable ones -- link loss,
    # pilot takeover, emergency escalation -- so they must not be the ones that
    # go unrecorded.
    record = {
        "state": state.name, "enable": "?", "mode": None, "armed": False,
        "alt": None, "vision_age": float("inf"), "n_persons": None,
        "error_x": 0.0, "error_y": 0.0, "bearing_deg": 0.0,
        "distance_cm": None, "yaw_rate": 0.0, "forward": 0.0, "gate": "-",
        "tick_s": tick_duration,
    }

    # ---- link health -------------------------------------------------------
    # Without this the loop keeps computing PID outputs and sending setpoints
    # into a dead port while reading a cached "GUIDED" forever.
    if not drone.link_ok():
        if fault_latch != "link":
            _log.error("MAVLink link lost (age %.1fs)", drone.link_age())
            fault_latch = "link"
        if state in (DroneState.TRACKING, DroneState.SEARCHING):
            soft_reset()
            enter_emergency("link lost")
        drone.reconnect()
        record["gate"] = "link_lost"
        return record
    if fault_latch == "link":
        _log.warning("MAVLink link restored")
        fault_latch = None

    mode = drone.get_mode()
    armed = drone.is_armed()
    alt = drone.get_relative_alt()

    enable = read_enable(now)
    # Snapshot the previous value and advance it immediately, so the edge test
    # below always compares against exactly last tick's value regardless of
    # which early return this tick takes.
    previous_enable = prev_enable
    prev_enable = enable

    record["mode"] = mode
    record["armed"] = armed
    record["alt"] = alt
    record["enable"] = _ENABLE_NAMES.get(enable, "?")

    # ---- vision ------------------------------------------------------------
    vision_ts, persons = camera.get_latest()
    vision_age = now - vision_ts
    # persons is None only if inference has never succeeded. An empty list means
    # inference ran and saw nobody, which is normal. Staleness is what makes a
    # retained detection unsafe, so the age check is the real gate here.
    vision_ok = persons is not None and vision_age <= cfg.VISION_MAX_AGE_S
    record["vision_age"] = vision_age
    record["n_persons"] = None if persons is None else len(persons)

    if state in (DroneState.TRACKING, DroneState.SEARCHING):
        if vision_ok:
            vision_faults = 0
        else:
            vision_faults += 1
            if vision_faults == 1:
                # frame_age distinguishes "the ISP pipeline is dead" from
                # "frames are arriving but inference has stopped".
                _log.warning(
                    "vision unhealthy: detection age %.2fs, frame age %.2fs, "
                    "persons=%s, health=%s",
                    vision_age, camera.frame_age(),
                    "None" if persons is None else len(persons),
                    camera.health(),
                )
            if vision_faults >= cfg.MAX_VISION_FAULTS:
                soft_reset()
                enter_emergency("vision stalled")

    # ---- pilot authority ---------------------------------------------------
    if mode != "GUIDED":
        if self_commanded_mode is not None and mode == self_commanded_mode:
            # A mode we asked for (RTL/LAND) is executing. The Pixhawk is
            # flying; send nothing and hold our state so the LED stays honest.
            record["gate"] = "self_" + mode.lower()
            return record

        # The pilot took control. This is the primary safety mechanism and it
        # works regardless of any Pi bug, because ArduPilot ignores guided
        # setpoints outside GUIDED. We deliberately send nothing here.
        if state != DroneState.IDLE:
            _log.info("pilot mode is %s -> IDLE", mode)
            soft_reset()
            state = DroneState.IDLE
        self_commanded_mode = None
        rtl_requested = False
        emergency_since = None
        engage_armed = False
        record["gate"] = "pilot"
        return record

    # ---- RC enable edge tracking ------------------------------------------
    if enable == OFF:
        engage_armed = False
    elif previous_enable == OFF:
        # Strict low->high transition. Latched so it survives until the other
        # engagement gates are satisfied. prev_enable starts as None, so a
        # switch already high at boot cannot engage until it is cycled.
        if not engage_armed:
            _log.info("AI enable edge seen (%s)", _ENABLE_NAMES[enable])
        engage_armed = True

    # The switch is checked in EVERY active state. Previously it was tested
    # only in READY, so flipping it off did not stop tracking.
    if enable == OFF and state in (DroneState.TRACKING, DroneState.SEARCHING):
        _log.warning("AI disabled by RC -> READY")
        soft_reset()
        state = DroneState.READY

    # ---- state machine ----------------------------------------------------
    if state == DroneState.IDLE:
        soft_reset()
        state = DroneState.READY
        _log.info("READY")
        drone.hover()

    elif state == DroneState.READY:
        drone.hover()

        target = tracker.select(persons) if vision_ok else None
        confirm_count = confirm_count + 1 if target is not None else 0

        alt_ok = alt is not None and alt >= cfg.MIN_TRACK_ALT_M
        target_ok = confirm_count >= cfg.TARGET_CONFIRM_FRAMES

        if engage_armed:
            if not armed:
                _status(now, "engage held: not armed")
            elif not alt_ok:
                _status(now, "engage held: altitude %s < %.1f m",
                        "unknown" if alt is None else "%.1f" % alt,
                        cfg.MIN_TRACK_ALT_M)
            elif not target_ok:
                _status(now, "engage held: target unconfirmed (%d/%d)",
                        confirm_count, cfg.TARGET_CONFIRM_FRAMES)
            else:
                soft_reset(keep_target=True)
                state = DroneState.TRACKING
                track_started = now
                last_confident_track = now
                engage_armed = False
                _log.info("TRACKING engaged (%s)", _ENABLE_NAMES[enable])

    elif state == DroneState.TRACKING:
        if not vision_ok:
            record.update(controller.hold())
        else:
            target = tracker.select(persons)

            if target is None:
                record.update(controller.hold())
                if lost_since is None:
                    lost_since = now
                if now - lost_since >= cfg.LOST_TRACK_S:
                    _log.info("target lost for %.1fs -> SEARCHING",
                              now - lost_since)
                    state = DroneState.SEARCHING
                    search_start = now
                    # Rotate toward the side the target was last seen. With the
                    # corrected sign, +error_x means the target was to the
                    # right, which needs a positive yaw rate.
                    search_dir = 1.0 if last_error_x >= 0 else -1.0
                    tracker.reset()
                    controller.reset()
                    reacquire_count = 0
            else:
                lost_since = None
                last_confident_track = now

                dist = distance_or_none(now)
                lock_center = tracker.get_lock_center(dist)
                locked, error_x, error_y = tracker.is_locked(target, lock_center)
                last_error_x = error_x

                # is_locked() now covers both axes, sized to the target's own
                # bounding box: the beam lands on the person if it falls
                # anywhere across their body, and the box measures exactly
                # that width. A fixed pixel gate ignored range entirely and
                # was several times tighter than the geometry requires.
                if locked:
                    update_distance(lidar.read_data(), now)
                    dist = distance_or_none(now)

                record.update(controller.update(
                    error_x, dist, yaw_only=(enable == YAW_ONLY),
                    target_width_px=target.width,
                ))
                record["error_y"] = error_y

                if (track_started is not None
                        and now - track_started > cfg.MAX_TRACK_DURATION_S):
                    enter_rtl("maximum tracking duration reached")

    elif state == DroneState.SEARCHING:
        target = tracker.select(persons) if vision_ok else None

        if target is not None:
            reacquire_count += 1
            if reacquire_count >= cfg.REACQUIRE_FRAMES:
                _log.info("target re-acquired after %d frames",
                          reacquire_count)
                state = DroneState.TRACKING
                lost_since = None
                reacquire_count = 0
                last_confident_track = now
                controller.reset()
                record["gate"] = "reacquired"
                return record
        else:
            reacquire_count = 0

        if search_start is None:
            # Defensive: the original would have raised TypeError on
            # `monotonic() - None` if SEARCHING was ever entered without the
            # TRACKING transition that set it.
            search_start = now

        search_yaw = cfg.SEARCH_YAW_RATE_DEG_S * search_dir
        drone.move(0.0, 0.0, 0.0, search_yaw)
        record["yaw_rate"] = search_yaw
        record["gate"] = "search"

        if now - search_start > cfg.SEARCH_TIMEOUT_S:
            enter_rtl("search timeout")
        elif (last_confident_track is not None
                and now - last_confident_track > cfg.MAX_LOST_TRACK_S):
            # Cumulative watchdog. Independent of the search timer and NOT
            # resettable by a transient re-acquisition, which is what let the
            # old SEARCHING<->TRACKING flip-flop postpone RTL indefinitely.
            enter_rtl("cumulative lost-track watchdog")

    elif state == DroneState.RTL:
        if not rtl_requested:
            rtl_requested = True
            drone.stop()
            if not request_mode("RTL"):
                enter_emergency("RTL request rejected")
        record["gate"] = "rtl"

    elif state == DroneState.LANDING:
        if self_commanded_mode != "LAND":
            drone.stop()
            if not request_mode("LAND"):
                enter_emergency("LAND request rejected")
        record["gate"] = "landing"

    elif state == DroneState.EMERGENCY:
        # Hovering forever ends in battery exhaustion and an uncontrolled
        # descent, so EMERGENCY escalates rather than parking.
        drone.stop()
        record["gate"] = "emergency"

        if emergency_since is None:
            emergency_since = now

        if now - emergency_since >= cfg.EMERGENCY_HOLD_S:
            if rtl_attempts < cfg.MAX_RTL_ATTEMPTS:
                rtl_attempts += 1
                _log.error("EMERGENCY: requesting RTL (%d/%d)",
                           rtl_attempts, cfg.MAX_RTL_ATTEMPTS)
                if request_mode("RTL"):
                    return record
            elif not land_attempted:
                land_attempted = True
                # A rejected RTL usually means no home or no position estimate,
                # and LAND needs neither.
                _log.error("EMERGENCY: RTL failed, requesting LAND")
                if request_mode("LAND"):
                    return record
            else:
                _status(now, "EMERGENCY: holding zero velocity; "
                             "pilot has the mode switch")

    return record


def main():
    global state, consecutive_faults, _slow_tick, fault_latch

    _setup_logging()
    _log.info("BILBO starting")

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    if not preflight():
        # No READY indication, and a distinct colour so the failure is visible
        # on the airframe rather than only in the log.
        led.led_status("solid", "orange")
        time.sleep(5.0)
        led.shutdown()
        return 1

    _open_csv()
    state = DroneState.IDLE
    tick_duration = 0.0

    try:
        while not _shutdown:
            tick_start = time.monotonic()
            _slow_tick = False

            record = None
            try:
                record = run_tick(tick_start, tick_duration)
                consecutive_faults = 0
                if fault_latch == "faults":
                    _log.warning("tick faults cleared")
                    fault_latch = None
                # Liveness counter. Incremented ONLY on a successful tick, so
                # if the loop stalls or faults the LED flash stops and the pilot
                # gets a visual cue for the otherwise-silent failure modes.
                led.note_tick()
            except Exception:
                consecutive_faults += 1
                _log.exception("tick fault %d", consecutive_faults)
                # Safe output FIRST, before any decision about escalation.
                try:
                    drone.stop()
                except Exception:
                    _log.exception("safe-stop send failed")
                if consecutive_faults >= cfg.MAX_CONSECUTIVE_FAULTS:
                    fault_latch = "faults"
                    enter_emergency("repeated tick faults")

            if record is None:
                # The tick raised. Still write a row so the gap is visible in
                # the flight record rather than silently absent.
                record = {"state": state.name, "gate": "exception",
                          "tick_s": tick_duration}
            _write_record(record, tick_start)

            try:
                led.render(state, fault=fault_latch)
            except Exception:
                _log.exception("LED render failed")

            tick_duration = time.monotonic() - tick_start
            remaining = cfg.TICK_PERIOD_S - tick_duration

            if remaining > 0:
                time.sleep(remaining)
            elif tick_duration > cfg.TICK_OVERRUN_LIMIT_S and not _slow_tick:
                # Something blocked. Escalate rather than flying on a frozen
                # loop -- this is the watchdog for the blocking failure modes.
                _log.error("tick overran %.3fs", tick_duration)
                enter_emergency("tick overrun")
    finally:
        _log.warning("shutting down: zero velocity, blanking LEDs")
        try:
            drone.stop()
        except Exception:
            _log.exception("final stop failed")
        try:
            led.shutdown()
        except Exception:
            _log.exception("LED shutdown failed")
        try:
            camera.stop()
        except Exception:
            _log.exception("camera stop failed")
        lidar.close()
        drone.disconnect_drone()
        if _csv_file is not None:
            try:
                _csv_file.flush()
                _csv_file.close()
            except Exception:
                _log.exception("CSV close failed")
        _log.info("BILBO stopped")

    return 0


if __name__ == "__main__":
    sys.exit(main())
