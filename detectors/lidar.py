"""TF Luna driver.

Public API is unchanged: read_data() returns a distance in centimetres or None.

The read path is rewritten because the original had five independent
flight-critical defects:

  1. No frame synchronisation. It read 9 bytes from wherever the stream sat.
     The sensor streams continuously before the port is opened, so the first
     read almost always landed mid-frame -- and every subsequent 9-byte read
     then started at the same wrong offset. Permanently misaligned with no
     resync path, so read_data() returned None for the entire flight.
  2. No checksum. Byte 8 is sum(bytes[0:8]) & 0xFF and was ignored, so a
     corrupted frame passed the two-byte header check and put a
     plausible-but-wrong distance straight into the PID at full authority.
  3. Oldest-frame-first. The sensor streams at 100 Hz and the loop runs at
     15 Hz, so ser.read(9) returned the OLDEST buffered frame and measurement
     latency grew without bound until the kernel buffer overflowed.
  4. No validity gating. Bytes 4-5 are signal strength and were ignored. Low
     strength is the dominant outdoor failure mode (bright sun, dark clothing).
  5. The port opened at import time with no try/except and no timeout, so
     merely importing this module could kill the process -- and it opened the
     same device as MAVLink, which is what prevented startup entirely.
"""

import logging

import serial

import config as cfg

_log = logging.getLogger(__name__)

ser = None
_buf = bytearray()

# Health counters for preflight and in-flight reporting.
_valid_frames = 0
_bad_checksums = 0
_rejected = 0
_last_strength = None    # signal strength of the last accepted frame


def init():
    """Open the LiDAR port. Raises on failure so preflight can abort loudly."""
    global ser, _buf, _valid_frames, _bad_checksums, _rejected, _last_strength

    if ser is not None:
        return

    _buf = bytearray()
    _valid_frames = 0
    _bad_checksums = 0
    _rejected = 0
    _last_strength = None

    # exclusive=True turns the old silent byte-stealing port conflict into an
    # immediate, obvious failure. timeout is set so no future refactor can
    # block the flight loop here.
    ser = serial.Serial(
        cfg.LIDAR_DEVICE,
        cfg.LIDAR_BAUD,
        timeout=0.05,
        exclusive=True,
    )
    # Discard whatever accumulated before we opened; it is stale by definition
    # and is the most likely source of an initial mid-frame offset.
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
    """Return the NEWEST checksum-valid distance in cm, or None.

    Never blocks: only bytes already buffered are consumed. Scanning forward
    and keeping the last valid frame means the freshest sample wins, so
    measurement latency is bounded by the loop period instead of growing.
    Advancing one byte at a time on a mismatch means a misaligned stream
    self-heals within a single frame.
    """
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
        # Header mismatch or bad checksum: advance one byte to resynchronise.
        i += 1

    # Retain any partial trailing frame for the next call.
    _buf = _buf[i:]
    return newest


def last_strength():
    """Signal strength of the last accepted frame, or None.

    Diagnostic only -- the flight path gates on strength inside _valid(). Low
    strength outdoors (bright sun, dark clothing) is the dominant TF Luna
    failure mode, so being able to watch this on a bench is worth having.
    """
    return _last_strength


def health():
    """Counters for logging and preflight diagnosis."""
    return {
        "valid": _valid_frames,
        "bad_checksum": _bad_checksums,
        "rejected": _rejected,
        "strength": _last_strength,
    }
