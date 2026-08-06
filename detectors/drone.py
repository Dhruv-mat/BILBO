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

VELOCITY_YAW_MASK = 0b010111000111
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


# Chatgpt told me to add these funtions to make sure debug is easy

def is_connected():
    return master is not None


def _resolve_device():
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


#connecting, discconecting and re connecting

def connect(timeout=10.0):
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
    try:
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL, 4, 1,
        )
    except Exception:
        _log.exception("request_data_stream failed")


#used to check if singal is established 
def send_heartbeat():
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



def get_mode():
    return _state["mode"]

def is_armed():
    return _state["armed"]

def get_relative_alt():
    return _state["rel_alt"]

def get_rc_channels():
    return dict(_state["rc"])

def get_channel(channel):
    return _state["rc"].get(channel)


# RC control stuff

def set_mode(mode, timeout=3.0, retries=2):

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

        _log.warning("mode change to %s not confirmed (attempt %d/%d)", mode, attempt + 1, retries + 1)

    _log.error("mode change to %s FAILED", mode)
    return False



# REALLLY DANGEROUS STUFFFF DONT CALL THESE COMMANDS IN FLIGHT 
def arm():

    if not is_connected():
        return False
    if not set_mode("GUIDED"):
        return False
    master.arducopter_arm()
    master.motors_armed_wait()
    _log.info("ARMED")
    return True


def disarm():

    if not is_connected():
        return False
    master.arducopter_disarm()
    master.motors_disarmed_wait()
    _log.info("DISARMED")
    return True



# you can call these bottom ones

def _sane(value, limit, name):

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




# we need the drone to move duhhhhhh

def move(forward_speed=0.0, right_speed=0.0, down_speed=0.0, yaw_rate=0.0,
         force=False):

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
            0.0, 0.0, 0.0,          
            fs, rs, ds,             
            0.0, 0.0, 0.0,         
            0.0,                   
            math.radians(yr),     
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
