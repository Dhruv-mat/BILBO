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

print("hi bitches")

# okay so this is basically gonna be a function that controls how much freedom the drone will hvae
OFF, YAW_ONLY, FULL = 0 ,1, 2
_ENABLE_NAMES = {OFF: "OFF", YAW_ONLY: "YAW_ONLY", FULL: "FULL"}


state = DroneState.IDLE

reacquire_count = 0
prev_enable = None
engage_armed = False      
confirm_count = 0           

lost_since = None          
last_error_x = 0.0
last_confident_track = None  
track_started = None

search_start = None
search_dir = 1.0           

last_dist = None
last_dist_t = None
jump_count = 0

vision_faults = 0

self_commanded_mode = None 
rtl_requested = False
emergency_since = None
rtl_attempts = 0
land_attempted = False

fault_latch = None
consecutive_faults = 0
_slow_tick = False          
_shutdown = False

_csv_file = None
_csv_writer = None
_last_csv_flush = 0.0
_last_status_log = 0.0



def _setup_logging(): 
    root = logging.getLogger()
    root .setLevel(getattr(logging, cfg.LOG_LEVEL, logging.INFO))

    fmt = logging.Formatter(
        "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
    )

    try:
        os.mkdir(cfg.LOG_DIR,exist_ok = True)
        handler = logging.handlers.RotatingFileHandler(
            os.path.join(cfg.LOG_DIR, "bilbo.log"),
            maxBytes=cfg.LOG_MAX_BYTES,
            backupCount=cfg.LOG_BACKUPS,
        )
        handler.setFormatter(fmt)
        root.addHandler(handler)

    except Exception:

        stream = logging.StreamHandler(sys.stderr)
        stream.setFormatter(fmt)
        root.addHandler(stream)
        root.error("file logging is not working, using stderr", exec_info = True)
        return

    if cfg.DEBUG_CONSOLE:
        stream = logging.StreamHandler(sys.stderr)
        stream.setFormatter(fmt)
        root.addHandler(stream)

CSV_COLUMNS = [
    "t", "state", "enable", "mode", "armed", "alt_m", "link_age_s",
    "rc_age_s", "vision_age_s", "n_persons", "error_x", "error_y",
    "bearing_deg", "dist_cm", "dist_age_s", "yaw_rate", "forward",
    "gate", "tick_s", "fault",

]

#-----the logging pant------------

# all of this is bieng done so i can data log and if needed change the values of that pid loop
def _open_csv():
    global _csv_file, _csv_writer
    try:
        path = os.path.join(cfg.LOG_DIR, "flight-%d.csv" % int(time.time()))
        _csv_file = open(path, "w", newline="", buffering=1024 * 64)
        _csv_writer = csv.writer(_csv_file)
        _csv_writer.writerow(CSV_COLUMNS)
        _log.info("flight log: %s", path)
    except Exception:
        _log.exception("could not open CSV flight log (sad stuffff)")
        _csv_file = None
        _csv_writer = None

def _fmt(value,spec):
    return "" if value is None else spec % value

# alright now we gotta write those csv values in some fancy prtty pretty formats 
def _write_record(record,now):
    global _last_csv_flush
    if _csv_writer is None:
        return


    # also i used chatgpt to generate this really fancy csv file (hope thats cool caue without this formatting im dead)
    try:
        _csv_writer.writerow([
            "%.3f" % now,
            record.get("state", ""),
            record.get("enable", ""),
            record.get("mode") or "",
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
        # till here heheheh

        if now - _last_csv_flush >= cfg.CSV_FLUSH_INTERVAL_S:
            _csv_file.flush()
            _last_csv_flush = now

    except Exception:
        _log.exception("CSV FILE DID NOT WORRKKK")


def _status(now, message, *args):
    global _last_status_log

    if now - _last_status_log < cfg.STATUS_LOG_INTERVAL_S:
        return
    _last_status_log = now
    _log.info(message, *args)

def _on_signal(signum,_frame):
    global _shutdown
    _shutdown = True
    _log.warning("signal %d received; shutting down", signum)



#----- reseting the PID LOOP PARTTTT---------

def soft_reset(keep_target = False):

    #this part helps setting the pid loops back to zero so no corrrections are pre stored

    global lost_since, reacquire_count, confirm_count, last_error_x
    global last_dist, last_dist_t, jump_count, vision_faults, track_started

    controller.reset()
    if not keep_target:
        tracker.reset()

    lost_since = 0
    reacquire_count = 0
    confirm_count = 0 
    last_error_x = 0
    last_dist = None
    last_dist_t = None
    jump_count = 0
    vision_faults = 0
    track_started = None


#------ really cool lidar logic--------

def update_dist(new_cm, now):
    if new_cm ==None:
        return

    # okay so this part is really sick, what i am trying to do here is making sure the jump in lidar is realistic and not an out of frame error
    if (last_dist is not None and abs(new_cm-last_dist) > cfg.LIDAR_MAX_JUMP_CM):
        jump_count +=1
        if jump_count < cfg.LIDAR_JUMP_CONFIRMS:
            _log.debug("rejecting range jump %d -> %d cm", last_dist, new_cm)
            return

    jump_count = 0
    last_dist = new_cm
    last_dist = now

def distance_or_none(now):

    if last_dist is None or last_dist_t is None:
        return None
    if now - last_dist > cfg.LIDAR_MAX_AGE_S:
        return None
    return last_dist


# this is just something i wanted for the csv column to see how old the lidar is, only used for debugging
def distance_age(now):
    if last_dist_t is None:
        return float("inf")
    return now - last_dist_t


#----- Writing the code to understand the RC funcs----------

def read_enable(now):
    "the 3 channel switching"
    raw = drone.get_channel(cfg.CH_ENABLE)
    if raw is None or drone.rc_age() > cfg.RC_STALE_S:
        return OFF
    if raw < cfg.SWITCH_MID_LOW_US:
        return OFF
    if raw < cfg.SWITCH_MID_HIGH_US:
        return YAW_ONLY
    return FULL

def request_mode(mode, timeout = 1.5, retries = 1):

    global _slow_tick, self_commanded_mode
    _slow_tick = True
    if drone.set_mode(mode, timeout=timeout, retries = retries):
        self_commanded_mode = mode
        return True
    return False



def enter_rtl(reason):
    global state, rtl_requested
    if state == DroneState.RTL:
        return
    _log.warning("RTL(%s)",reason)
    state = DroneState.RTL
    rtl_requested = False


def enter_emegency(reason):
    global state, emergency_since, rtl_attempts, land_attempted
    if state == DroneState.EMERGENCY:
        return
    state = DroneState.EMERGENCY
    emergency_since = None
    rtl_attempts = 0
    land_attempted = False


#------preflight checks and weird ass things--------

def _wait_for(predicate, timeout, interval=0.05, pump=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pump is not None:
            pump()
        try:
            if predicate():
                return True
        except Exception:
            _log.exception("preflight issues happended")


# this one is really important cause it checks if everything is working and fineee before the flight 

def preflight():

    led.init()
    led.selftest()

    if not drone.connect(timeout = 10.0):
        _log.error("Preflight Fail: Mavlink ain't working")
        return False

    try:
        lidar.init()
    except Exception:
        _log.error("Preflight Fail: LiDAR ain't working")
        return False
    
    valid = {'n':0}

    def lidar_healthy():
        if lidar.read_data() is not None:
            valid["n"] += 1
        return valid["n"] >= cfg.LIDAR_HEALTH_FRAMES


    if not _wait_for(lidar_healthy, timeout=3.0):
        _log.error("Preflight Fail:LiDAR gave %d/%d valid frames (%s)",
                   valid["n"], cfg.LIDAR_HEALTH_FRAMES, lidar.health())

        return False

    try:
        width, height = camera.intitalise()
    except Exception:
        _log.exception("Preflight Fail: camera did not start")
        return False

    if not _wait_for(camera.inference_alive, timeout=8.0):
        _log.error("Preflight Fail: no inference output from the IMX500")
        return False

    try:
        tracker.configure(width, height)
    except Exception:
        _log.exception("PREFLIGHT FAIL: camera geometry mismatch")
        return False


    if not _wait_for( lambda: drone.get_channel(cfg.CH_ENABLE) is not None, timeout=5.0,pump=drone.poll):
        _log.error("Preflight Fail: no RC input detected, could never switch to guided")
        return False

    _log.info("preflight OK (lidar %s, camera %s)",
              lidar.health(), camera.health())
    return True


#-----live checks--------

#so basically this part of the code is gonna keep checking if everything is still working after preflight

def run_tick(now, tick_duration):
    global state, prev_enable, engage_armed, confirm_count
    global lost_since, reacquire_count, last_error_x, last_confident_track
    global track_started, vision_faults, fault_latch
    global self_commanded_mode, rtl_requested
    global emergency_since, rtl_attempts, land_attempted
    global search_start, search_dir


    record = {
        "state": state.name, "enable": "?", "mode": None, "armed": False,
        "alt": None, "vision_age": float("inf"), "n_persons": None,
        "error_x": 0.0, "error_y": 0.0, "bearing_deg": 0.0,
        "distance_cm": None, "yaw_rate": 0.0, "forward": 0.0, "gate": "-",
        "tick_s": tick_duration,
    }


    drone.poll()
    drone.send_heartbeat()

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

    previous_enable = prev_enable
    prev_enable = enable

    record["mode"] = mode
    record["armed"] = armed
    record["alt"] = alt
    record["enable"] = _ENABLE_NAMES.get(enable, "?")

    vision_ts, persons = camera.get_latest()
    vision_age = now - vision_ts

    vision_ok = persons is not None and vision_age <= cfg.VISION_MAX_AGE_S
    record["vision_age"] = vision_age
    record["n_persons"] = None if persons is None else len(persons)

    if state in (DroneState.TRACKING, DroneState.SEARCHING):
        if vision_ok:
            vision_faults = 0
        else:
            vision_faults += 1
            if vision_faults == 1:
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


    if mode != "GUIDED":
        if self_commanded_mode is not None and mode == self_commanded_mode:
            record["gate"] = "self_" + mode.lower()
            return record

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

    if enable == OFF:
        engage_armed = False
    elif previous_enable == OFF:
        if not engage_armed:
            _log.info("AI enable edge seen (%s)", _ENABLE_NAMES[enable])
        engage_armed = True


    if enable == OFF and state in (DroneState.TRACKING, DroneState.SEARCHING):
        _log.warning("AI disabled by RC -> READY")
        soft_reset()
        state = DroneState.READY



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








    
    




while True:

    ch6 = drone.get_channel(6)
    mode = drone.get_mode()

    if mode != "GUIDED":
        drone.stop()
        state = DroneState.IDLE
        continue

    if state == DroneState.READY:
        if ch6 is not None and ch6 > SWITCH_HIGH:
            print("Tracking enabled")
            led.led_status(effect="solid", color="blue")
            state = DroneState.TRACKING
            continue

    elif state == DroneState.TRACKING:

        persons = camera.get_people()
        target = tracker.select(persons)
        led.led_status(effect="solid", color="green")

        if target is not None:
            lost_frames = 0

        if target is None:
            lost_frames += 1

            if lost_frames >= LOST_FRAME_LIMIT:
                print("starting up searching")
                search_start_time = time.monotonic()
                state = DroneState.SEARCHING
            continue


        if last_good_distance is None:
            lock_center = CAMERA_CENTER
        else:
            lock_center = tracker.get_lock_center(last_good_distance)
        locked, error_x = tracker.is_locked(target, lock_center)

        if abs(error_x) < controller.YAW_DEADBAND:
            
            new_distance = lidar.read_data()
            if new_distance is not None:
                last_good_distance = new_distance

        controller.update(error_x, last_good_distance)

    elif state == DroneState.SEARCHING:

        print("Searching...")
        led.led_status(effect="blink", color="yellow")
        persons = camera.get_people()
        target = tracker.select(persons)

        if target is not None:
            print("Target found!")
            state = DroneState.TRACKING
            continue

        drone.move(
            yaw_rate=SEARCH_YAW_RATE
        )

        if time.monotonic() - search_start_time > SEARCH_TIMEOUT:
            state = DroneState.RTL

    elif state == DroneState.IDLE:
        led.led_status(effect="solid", color="white")
        if mode == "GUIDED":
            print("READY")
            state = DroneState.READY

        continue

    elif state == DroneState.RTL:
        print("RTL")
        led.led_status(effect="solid", color="purple")
        drone.set_mode("RTL")
        continue

    elif state == DroneState.EMERGENCY:
        drone.hover()
        led.led_status(effect="blink", color="red")
        continue


