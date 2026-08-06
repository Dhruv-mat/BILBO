"""Target selection and image-space error computation.

Kept deliberately lightweight: no model, no filter bank, no new dependencies.
The one behavioural change is target *persistence*.

The original select() called max(persons, key=area) independently every frame,
with no notion of identity. Largest area means closest person, so any stranger
walking between the drone and the target instantly stole the lock and the drone
flew at them. With two people at similar range the selection flickered frame to
frame, error_x jumped by hundreds of pixels, and yaw slammed back and forth.
"""

import logging
import math

import config as cfg

_log = logging.getLogger(__name__)

# The currently tracked person, carried across frames.
_target = None
_misses = 0        # consecutive frames the target was not re-associated


def configure(width_px, height_px):
    """Validate the real camera geometry against config.

    The original hardcoded IMAGE_WIDTH = 640 here while main.py derived the
    centre from the actual stream size. If the stream was not 640 wide, the
    lock centre jumped by (width - 640)/2 pixels the instant the first LiDAR
    reading arrived -- a step in error_x and a sudden yaw kick. Rather than
    plumb the value through two modules, this asserts the single source of
    truth and fails loudly on mismatch.
    """
    if width_px != cfg.IMAGE_WIDTH_PX or height_px != cfg.IMAGE_HEIGHT_PX:
        raise RuntimeError(
            "camera geometry %dx%d does not match config %dx%d -- update "
            "IMAGE_WIDTH_PX / IMAGE_HEIGHT_PX (PIXELS_PER_DEGREE and the "
            "LiDAR boresight row depend on them)"
            % (width_px, height_px, cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)
        )
    _log.info("tracker geometry %dx%d, %.2f px/deg",
              width_px, height_px, cfg.PIXELS_PER_DEGREE)


def reset():
    """Forget the current target so the next select() re-designates."""
    global _target, _misses
    if _target is not None:
        _log.info("tracker target cleared")
    _target = None
    _misses = 0


def has_target():
    return _target is not None


def get_lock_center(distance_cm):
    """Image column the target should sit on for the LiDAR to be on it.

    Corrects for the horizontal offset between the camera axis and the LiDAR.
    The original divided BASELINE = 0.05 (metres) by a distance in centimetres,
    yielding a 0.15 px correction where the true parallax for a 5 cm baseline at
    1.6 m is ~1.79 deg or ~14.7 px -- 100x too small, so the correction was
    effectively absent. It matters now that the range gate is sized to the
    target box and can be as tight as LIDAR_HGATE_MIN_PX.
    """
    center = cfg.IMAGE_WIDTH_PX / 2.0
    # NaN fails every comparison, so `distance_cm <= 0` does NOT catch it: a NaN
    # range would sail through and return a NaN lock centre, making error_x NaN
    # and poisoning everything downstream. drone.move() would still clamp it to
    # zero at the wire, but a lock centre is not the place to rely on that.
    if distance_cm is None or not math.isfinite(distance_cm):
        return center
    if distance_cm <= 0:
        return center
    angle_deg = math.degrees(math.atan(cfg.LIDAR_BASELINE_CM / distance_cm))
    return center + cfg.LIDAR_OFFSET_SIGN * angle_deg * cfg.PIXELS_PER_DEGREE


def _clamp(value, low, high):
    return max(low, min(high, value))


def hgate_px(box_width_px):
    """Horizontal range-gate half-width, proportional to the target's box.

    The beam lands on the person if it falls anywhere across their body, and the
    box width is exactly that width in pixels. A fixed gate ignored this and was
    several times tighter than the geometry requires.
    """
    return _clamp(box_width_px * cfg.LIDAR_HGATE_FRAC,
                  cfg.LIDAR_HGATE_MIN_PX, cfg.LIDAR_HGATE_MAX_PX)


def vgate_px(box_height_px):
    """Vertical range-gate half-height: keep the beam on the torso."""
    return _clamp(box_height_px * cfg.LIDAR_VGATE_FRAC,
                  cfg.LIDAR_VGATE_MIN_PX, cfg.LIDAR_VGATE_MAX_PX)


def is_locked(person, lock_center_x):
    """Return (locked, error_x, error_y) in pixels.

    `locked` now covers BOTH axes, so callers do not need a second vertical
    check. error_y is measured against the LiDAR's boresight row, not the image
    centre: without a vertical condition, flying at altitude means the forward
    beam shoots over the person's head while the camera still sees them fine.
    """
    if person is None or lock_center_x is None:
        return False, 0.0, 0.0

    error_x = person.center_x - lock_center_x
    error_y = person.center_y - cfg.LIDAR_BORESIGHT_ROW_PX
    locked = (abs(error_x) < hgate_px(person.width)
              and abs(error_y) < vgate_px(person.height))
    return locked, error_x, error_y


def select(persons):
    """Return the tracked person, or None if the target was not re-found.

    None means "this frame did not contain our target" and is counted as a lost
    frame by the caller. It does not mean "no people in view".
    """
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
        # Designation: largest box, i.e. the closest person. Only used to pick
        # the initial target at engage time; from then on identity is carried
        # by association below.
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
            # Self-heal. Without this the tracker compares every future frame
            # against a stale position forever: once the person has moved beyond
            # TRACK_GATE_PX of where they used to be, they can never
            # re-associate and the target is lost permanently.
            _target = max(persons, key=lambda p: p.area)
            _log.info("re-designating after %d failed associations "
                      "(x=%.0f y=%.0f)",
                      _misses, _target.center_x, _target.center_y)
            _misses = 0
            return _target
        # Brief gap. Better to report a miss than to lock onto the wrong person.
        return None

    _target = min(candidates, key=distance_to_prev)
    _misses = 0
    return _target


def misses():
    """Consecutive frames the target could not be re-associated. Diagnostic."""
    return _misses
