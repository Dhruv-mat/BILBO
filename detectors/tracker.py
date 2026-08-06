import logging
import math

import config as cfg

_log = logging.getLogger(__name__)

# The currently tracked person, carried across frames.
_target = None
_misses = 0       


def configure(width_px, height_px):
    if width_px != cfg.IMAGE_WIDTH_PX or height_px != cfg.IMAGE_HEIGHT_PX:
        raise RuntimeError( "camera geometry %dx%d does not match config %dx%d -- update "
            "IMAGE_WIDTH_PX / IMAGE_HEIGHT_PX (PIXELS_PER_DEGREE and the "
            "LiDAR boresight row depend on them)"
            % (width_px, height_px, cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)
        )
    _log.info("tracker geometry %dx%d, %.2f px/deg",
              width_px, height_px, cfg.PIXELS_PER_DEGREE)


def reset():
    global _target, _misses
    if _target is not None:
        _log.info("tracker target cleared")
    _target = None
    _misses = 0


def has_target():
    return _target is not None

def get_lock_center(distance_cm):
    center = cfg.IMAGE_WIDTH_PX / 2.0
    if distance_cm is None or distance_cm <= 0:
        return center
    angle_deg = math.degrees(math.atan(cfg.LIDAR_BASELINE_CM / distance_cm))
    return center + cfg.LIDAR_OFFSET_SIGN * angle_deg * cfg.PIXELS_PER_DEGREE


def _clamp(value, low, high):
    return max(low, min(high, value))


def hgate_px(box_width_px):
    return _clamp(box_width_px * cfg.LIDAR_HGATE_FRAC,
                  cfg.LIDAR_HGATE_MIN_PX, cfg.LIDAR_HGATE_MAX_PX)


def vgate_px(box_height_px):
    """Vertical range-gate half-height: keep the beam on the torso."""
    return _clamp(box_height_px * cfg.LIDAR_VGATE_FRAC,
                  cfg.LIDAR_VGATE_MIN_PX, cfg.LIDAR_VGATE_MAX_PX)


def is_locked(person, lock_center_x):

    if person is None or lock_center_x is None:
        return False, 0.0, 0.0

    error_x = person.center_x - lock_center_x
    error_y = person.center_y - cfg.LIDAR_BORESIGHT_ROW_PX
    locked = (abs(error_x) < hgate_px(person.width)
              and abs(error_y) < vgate_px(person.height))
    return locked, error_x, error_y


def select(persons):
    global _target, _misses

    if not persons:
        _misses += 1
        if _misses >= cfg.TRACK_MAX_MISSES and _target is not None:
            _log.info("target dropped after %d misses; will re-designate",
                      _misses)
            _target = None
        return None

    prev = _target

    if prev is None:
        _target = max(persons, key=lambda p: p.area)
        _misses = 0
        _log.info("target designated at x=%.0f y=%.0f area=%d",
                  _target.center_x, _target.center_y, _target.area)
        return _target

    def distance_to_prev(p):
        return math.hypot(p.center_x - prev.center_x,
                          p.center_y - prev.center_y)

    prev_area = max(prev.area, 1)
    candidates = [
        p for p in persons
        if distance_to_prev(p) < cfg.TRACK_GATE_PX
        and (1.0 / cfg.TRACK_AREA_RATIO)
        <= (p.area / prev_area)
        <= cfg.TRACK_AREA_RATIO
    ]

    if not candidates:
        _misses += 1
        if _misses >= cfg.TRACK_MAX_MISSES:
           
            _target = max(persons, key=lambda p: p.area)
            _log.info("redesignating after %d failed associations "
                      "(x=%.0f y=%.0f)",
                      _misses, _target.center_x, _target.center_y)
            _misses = 0
            return _target
        return None

    _target = min(candidates, key=distance_to_prev)
    _misses = 0
    return _target


def misses():
    """Consecutive frames the target could not be re-associated. Diagnostic."""
    return _misses
