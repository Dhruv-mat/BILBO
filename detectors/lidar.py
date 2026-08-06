

import logging
import serial
import config as cfg

_log = logging.getLogger(__name__)
ser = None
_buf = bytearray()


_valid_frames = 0
_bad_checksums = 0
_rejected = 0
_last_strength = None   


#new fancy initialisation command i got of the internet
def init():
    global ser, _buf, _valid_frames, _bad_checksums, _rejected, _last_strength
    if ser is not None:
        return

    _buf = bytearray()
    _valid_frames = 0
    _bad_checksums = 0
    _rejected = 0
    _last_strength = None
    ser = serial.Serial(
        cfg.LIDAR_DEVICE,
        cfg.LIDAR_BAUD,
        timeout=0.05,
        exclusive=True,
    )
    ser.reset_input_buffer()
    _log.info("LiDAR open on %s @ %d", cfg.LIDAR_DEVICE, cfg.LIDAR_BAUD)


def close():
    global ser
    if ser is None:
        return
    try:
        ser.close()
    except Exception:
        _log.exception("LiDAR close failed")
    finally:
        ser = None


def is_open():
    return ser is not None


def _valid(dist_cm, strength):
    return (
        cfg.LIDAR_MIN_CM <= dist_cm <= cfg.LIDAR_MAX_CM
        and cfg.LIDAR_MIN_STRENGTH <= strength < 65535
    )


def read_data():
    global _buf, _valid_frames, _bad_checksums, _rejected, _last_strength

    if ser is None:
        return None

    try:
        pending = ser.in_waiting
        if pending:
            _buf += ser.read(pending)
    except Exception:
        _log.exception("LiDAR read failed")
        return None

    if len(_buf) > cfg.LIDAR_RX_BUFFER_CAP:
        _buf = _buf[-cfg.LIDAR_RX_BUFFER_CAP:]

    newest = None
    i = 0
    while i + 9 <= len(_buf):
        if _buf[i] == 0x59 and _buf[i + 1] == 0x59:
            frame = _buf[i:i + 9]
            if (sum(frame[0:8]) & 0xFF) == frame[8]:
                dist = frame[2] | (frame[3] << 8)
                strength = frame[4] | (frame[5] << 8)
                if _valid(dist, strength):
                    newest = dist
                    _last_strength = strength
                    _valid_frames += 1
                else:
                    _rejected += 1
                i += 9
                continue
            _bad_checksums += 1
        i += 1

    _buf = _buf[i:]
    return newest


def last_strength():
    return _last_strength


# at any given time as i saw from a refrence repo it is good to keep a tab on the lidar health so i implemented that in my sequence asw
def health():
    return {
        "valid": _valid_frames,
        "bad_checksum": _bad_checksums,
        "rejected": _rejected,
        "strength": _last_strength,
    }
