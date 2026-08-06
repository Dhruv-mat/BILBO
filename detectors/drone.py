"""MAVLink link layer to the Pixhawk.

The Pi sends body-frame velocity + yaw-rate setpoints and nothing else. The
Pixhawk owns stabilisation, EKF, GPS, motor mixing, compass, battery/RC
failsafes and RTL execution. Nothing here changes that split.

Key structural change: all inbound traffic is drained ONCE per control tick
into a cache by poll(), and every consumer reads the cache. The original
get_mode() performed no receive of its own -- it read master.flightmode, which
only updated as an invisible side effect of get_rc_channels() having drained the
buffer immediately beforehand. Reordering those two calls would have silently
started returning stale data.
"""

import glob
import logging
import math
import os
import time

from pymavlink import mavutil

import config as cfg

_log = logging.getLogger(__name__)

master = None

# Bit 11 (yaw_rate) = 0 -> used. Bit 10 (yaw) = 1 -> ignored. Bits 3-5
# (velocity x/y/z) = 0 -> used. Bits 0-2 (position) and 6-8 (acceleration) = 1
# -> ignored. This is the canonical velocity + yaw-rate mask (1479 decimal) and
# it is correct. vz is deliberately *enabled* and always sent as 0, which is an
# explicit zero climb rate rather than an unspecified one.
VELOCITY_YAW_MASK = 0b010111000111

# MAV_FRAME_BODY_NED (8) is accepted by ArduPilot and rotated by yaw only, so vx
# is horizontal-plane forward regardless of pitch attitude -- which is what this
# controller wants -- and vz is unaffected by that rotation. The MAVLink spec
# deprecates it in favour of BODY_FRD (12), but ArduPilot still handles BODY_NED
# and swapping a working frame immediately pre-flight is risk without gain.
VELOCITY_FRAME = mavutil.mavlink.MAV_FRAME_BODY_NED

_state = {
    "mode": None,
    "armed": False,
    "rc": {},
    "rel_alt": None,
    "last_heartbeat": 0.0,
    "last_rc": 0.0,
}

_last_setpoint_tx = 0.0
_last_heartbeat_tx = 0.0
_last_reconnect = 0.0


# --------------------------------------------------------------- helpers ----

def is_connected():
    return master is not None


def _resolve_device():
    """Return the MAVLink device path, or None.

    Prefers the exact configured by-id path. Falls back to globbing by-id and
    requires EXACTLY ONE match: one match is unambiguous so the anti-swap
    guarantee holds, while zero or several is a hard failure rather than a
    guess at which adapter is the flight controller.
    """
    if os.path.exists(cfg.MAVLINK_DEVICE):
        return cfg.MAVLINK_DEVICE

    matches = []
    for pattern in cfg.MAVLINK_DEVICE_GLOBS:
        matches.extend(glob.glob(pattern))
    matches = sorted(set(matches))

    if len(matches) == 1:
        _log.warning("MAVLINK_DEVICE not found; using sole by-id match %s",
                     matches[0])
        return matches[0]

    available = sorted(glob.glob("/dev/serial/by-id/*"))
    _log.error(
        "cannot resolve MAVLink device. configured=%s, glob matches=%s, "
        "available by-id=%s",
        cfg.MAVLINK_DEVICE, matches, available,
    )
    return None


def _ingest(msg):
    """Fold one received message into the cache."""
    msg_type = msg.get_type()

    if msg_type == "HEARTBEAT":
        _state["last_heartbeat"] = time.monotonic()
        _state["mode"] = master.flightmode
        _state["armed"] = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )
    elif msg_type == "RC_CHANNELS":
        _state["rc"] = {
            i: getattr(msg, "chan%d_raw" % i) for i in range(1, 9)
        }
        _state["last_rc"] = time.monotonic()
    elif msg_type == "GLOBAL_POSITION_INT":
        _state["rel_alt"] = msg.relative_alt / 1000.0


def poll(max_msgs=200):
    """Drain all pending traffic into the cache. Call once per control tick.

    Bounded by max_msgs so a flooded link cannot make this never return -- an
    unbounded drain would be an infinite loop in the flight path.
    """
    if not is_connected():
        return 0

    count = 0
    for _ in range(max_msgs):
        try:
            msg = master.recv_match(blocking=False)
        except Exception:
            _log.exception("recv_match failed")
            break
        if msg is None:
            break
        count += 1
        if msg.get_type() == "BAD_DATA":
            continue
        try:
            _ingest(msg)
        except Exception:
            _log.exception("failed to ingest %s", msg.get_type())
    return count


# ------------------------------------------------------------ connection ----

def connect(timeout=10.0):
    """Open the link and wait for a heartbeat. Returns True on success.

    The original called wait_heartbeat() with no timeout, so an absent or
    misconfigured Pixhawk produced an unkillable hang after the camera was
    already running.
    """
    global master

    device = _resolve_device()
    if device is None:
        return False

    _log.info("connecting to Pixhawk on %s @ %d", device, cfg.MAVLINK_BAUD)
    try:
        candidate = mavutil.mavlink_connection(device, baud=cfg.MAVLINK_BAUD)
    except Exception:
        _log.exception("failed to open %s", device)
        return False

    try:
        hb = candidate.wait_heartbeat(timeout=timeout)
    except Exception:
        _log.exception("wait_heartbeat raised")
        hb = None

    if hb is None:
        _log.error("no heartbeat within %.1fs on %s", timeout, device)
        try:
            candidate.close()
        except Exception:
            pass
        return False

    master = candidate
    _state["last_heartbeat"] = time.monotonic()
    _state["mode"] = master.flightmode
    _log.info("heartbeat OK: system %d component %d, mode %s",
              master.target_system, master.target_component, master.flightmode)
    request_streams()
    return True


def disconnect_drone():
    global master
    if master is None:
        return
    try:
        master.close()
    except Exception:
        _log.exception("close failed")
    finally:
        master = None


def reconnect():
    """One bounded reconnect attempt. Returns True on success.

    Rate limited internally so it can be called from the fault path every tick
    without hammering the port, and uses a short heartbeat timeout so the
    control loop keeps ticking.
    """
    global master, _last_reconnect

    now = time.monotonic()
    if now - _last_reconnect < cfg.RECONNECT_INTERVAL_S:
        return False
    _last_reconnect = now

    disconnect_drone()

    device = _resolve_device()
    if device is None:
        return False

    try:
        candidate = mavutil.mavlink_connection(device, baud=cfg.MAVLINK_BAUD)
        if candidate.wait_heartbeat(timeout=0.5) is None:
            candidate.close()
            return False
    except Exception:
        _log.exception("reconnect failed")
        return False

    master = candidate
    _state["last_heartbeat"] = time.monotonic()
    _state["mode"] = master.flightmode
    _log.warning("MAVLink link re-established on %s", device)
    request_streams()
    return True


def request_streams():
    """Ask for the messages we depend on at known rates.

    RC_CHANNELS is not guaranteed to stream on a non-GCS serial port; the
    SRn_* parameters for that port govern it and several default to 0. If it
    never arrives, get_channel() returns None forever and the drone can never
    leave READY.
    """
    if not is_connected():
        return

    wanted = (
        (mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2),
        (mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS, 10),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 4),
        (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2),
    )
    for msg_id, hz in wanted:
        try:
            master.mav.command_long_send(
                master.target_system, master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                msg_id, int(1e6 / hz), 0, 0, 0, 0, 0,
            )
        except Exception:
            _log.exception("SET_MESSAGE_INTERVAL failed for id %d", msg_id)

    # Belt-and-braces for firmware that ignores SET_MESSAGE_INTERVAL.
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
        )
    except Exception:
        _log.exception("request_data_stream failed")


# ------------------------------------------------------------- heartbeat ----

def send_heartbeat():
    """Emit a companion heartbeat at most once per HEARTBEAT_TX_INTERVAL_S.

    Called from the control loop, deliberately NOT from a background thread: a
    threaded heartbeat would keep reassuring the Pixhawk that the companion is
    healthy while the flight loop was hung, which is actively harmful. Emitting
    it here makes heartbeat liveness and loop liveness the same thing. It also
    keeps this module single-writer, so MAVLink frames cannot interleave and
    corrupt without a TX lock.
    """
    global _last_heartbeat_tx

    if not is_connected():
        return False

    now = time.monotonic()
    if now - _last_heartbeat_tx < cfg.HEARTBEAT_TX_INTERVAL_S:
        return False

    try:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0,
        )
    except Exception:
        _log.exception("heartbeat send failed")
        return False

    _last_heartbeat_tx = now
    return True


def link_age():
    """Seconds since the last HEARTBEAT from the Pixhawk."""
    if not is_connected() or _state["last_heartbeat"] == 0.0:
        return float("inf")
    return time.monotonic() - _state["last_heartbeat"]


def link_ok():
    return link_age() <= cfg.LINK_TIMEOUT_S


def rc_age():
    """Seconds since the last RC_CHANNELS message."""
    if _state["last_rc"] == 0.0:
        return float("inf")
    return time.monotonic() - _state["last_rc"]


# ----------------------------------------------------------- cache reads ----

def get_mode():
    return _state["mode"]


def is_armed():
    return _state["armed"]


def get_relative_alt():
    """Relative altitude in metres, or None if never reported.

    Single accessor on purpose: a second `get_altitude()` alias used to exist
    here, and two names for one value is a maintenance hazard -- someone edits
    one and the other silently keeps the old behaviour.
    """
    return _state["rel_alt"]


def get_rc_channels():
    return dict(_state["rc"])


def get_channel(channel):
    return _state["rc"].get(channel)


# ------------------------------------------------------------ mode change ----

def set_mode(mode, timeout=3.0, retries=2):
    """Request a flight mode. Returns True only if the vehicle confirms it.

    The original waited on master.flightmode in a loop that called only
    time.sleep(). pymavlink is synchronous -- flightmode only updates when a
    HEARTBEAT is parsed, and that loop parsed nothing, so the condition could
    never become true. It blocked forever, unconditionally, with no timeout.
    Reached on every RTL.

    Received messages are fed through _ingest() rather than filtered, so RC and
    telemetry are not discarded during the wait.
    """
    if not is_connected():
        _log.error("set_mode(%s): not connected", mode)
        return False

    try:
        modes = master.mode_mapping()
    except Exception:
        _log.exception("mode_mapping failed")
        return False

    if not modes or mode not in modes:
        _log.error("set_mode(%s): unknown mode (known: %s)",
                   mode, sorted(modes) if modes else None)
        return False

    mode_id = modes[mode]

    for attempt in range(retries + 1):
        try:
            master.set_mode(mode_id)
        except Exception:
            _log.exception("set_mode send failed")
            return False

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                msg = master.recv_match(blocking=True, timeout=0.25)
            except Exception:
                _log.exception("recv during set_mode failed")
                msg = None
            if msg is not None and msg.get_type() != "BAD_DATA":
                try:
                    _ingest(msg)
                except Exception:
                    _log.exception("ingest during set_mode failed")
            if _state["mode"] == mode:
                _log.info("mode is now %s", mode)
                return True

        _log.warning("mode change to %s not confirmed (attempt %d/%d)",
                     mode, attempt + 1, retries + 1)

    _log.error("mode change to %s FAILED", mode)
    return False


def arm():
    """Not used by the flight loop -- the pilot arms manually. Retained for
    bench use only."""
    if not is_connected():
        return False
    if not set_mode("GUIDED"):
        return False
    master.arducopter_arm()
    master.motors_armed_wait()
    _log.info("ARMED")
    return True


def disarm():
    """Never called in flight. Software must not stop motors above ground --
    that drops the aircraft, and ArduPilot refuses it while flying anyway."""
    if not is_connected():
        return False
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    _log.info("DISARMED")
    return True


# --------------------------------------------------------------- setpoints ----

def _sane(value, limit, name):
    """Final clamp before the wire.

    Independent of the PID output_limits so no upstream bug can emit an absurd
    velocity, and NaN/inf is caught at the last possible moment -- a non-finite
    velocity reaching the Pixhawk is genuinely dangerous.
    """
    if value is None:
        return 0.0
    try:
        value = float(value)
    except (TypeError, ValueError):
        _log.error("non-numeric %s=%r -> 0", name, value)
        return 0.0
    if not math.isfinite(value):
        _log.error("non-finite %s=%r -> 0", name, value)
        return 0.0
    if value > limit:
        return limit
    if value < -limit:
        return -limit
    return value


def move(forward_speed=0.0, right_speed=0.0, down_speed=0.0, yaw_rate=0.0,
         force=False):
    """Send one body-frame velocity + yaw-rate setpoint. Returns True if sent.

    forward_speed now has a default: the original required it positionally, so
    drone.move(yaw_rate=...) in the SEARCHING branch raised TypeError and
    entering SEARCHING killed the process.

    Rate limited because nothing previously regulated send rate. At 57600 baud
    roughly 88 of these 65-byte messages fit in a second, while a tight loop
    attempts thousands -- the TX buffer saturates and latency grows without
    bound, so the Pixhawk acts on setpoints that are seconds stale.

    force=True bypasses the rate limit for safe-stop paths, where dropping the
    zero-velocity command would be worse than a little extra traffic.
    """
    global _last_setpoint_tx

    if not is_connected():
        return False

    now = time.monotonic()
    if not force and now - _last_setpoint_tx < cfg.MIN_SETPOINT_INTERVAL_S:
        return False

    fs = _sane(forward_speed, cfg.HARD_MAX_SPEED_MS, "forward_speed")
    rs = _sane(right_speed, cfg.HARD_MAX_SPEED_MS, "right_speed")
    ds = _sane(down_speed, cfg.HARD_MAX_SPEED_MS, "down_speed")
    yr = _sane(yaw_rate, cfg.HARD_MAX_YAW_RATE_DEG_S, "yaw_rate")

    try:
        master.mav.set_position_target_local_ned_send(
            0,
            master.target_system,
            master.target_component,
            VELOCITY_FRAME,
            VELOCITY_YAW_MASK,
            0.0, 0.0, 0.0,          # position (ignored)
            fs, rs, ds,             # velocity, body frame, m/s
            0.0, 0.0, 0.0,          # acceleration (ignored)
            0.0,                    # yaw (ignored)
            math.radians(yr),       # yaw rate, rad/s; +ve = nose right
        )
    except Exception:
        _log.exception("setpoint send failed")
        return False

    _last_setpoint_tx = now
    return True


def hover(force=False):
    """Explicit zero velocity, zero yaw rate."""
    return move(0.0, 0.0, 0.0, 0.0, force=force)


def stop(force=True):
    """Safe stop. Forced by default -- this is the command we must not drop."""
    return hover(force=force)
