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
    global _target
    if _target is not None:
        _log.info("tracker target cleared")
    _target = None


def has_target():
    return _target is not None


def get_lock_center(distance_cm):
    """Image column the target should sit on for the LiDAR to be on it.

    Corrects for the horizontal offset between the camera axis and the LiDAR.
    The original divided BASELINE = 0.05 (metres) by a distance in centimetres,
    yielding a 0.15 px correction where the true parallax for a 5 cm baseline at
    1.6 m is ~1.79 deg or ~14.7 px -- 100x too small, so the correction was
    effectively absent. That was harmless only while the gate was 50 px wide;
    it becomes live now that the gate is 15 px.
    """
    center = cfg.IMAGE_WIDTH_PX / 2.0
    if distance_cm is None or distance_cm <= 0:
        return center
    angle_deg = math.degrees(math.atan(cfg.LIDAR_BASELINE_CM / distance_cm))
    return center + cfg.LIDAR_OFFSET_SIGN * angle_deg * cfg.PIXELS_PER_DEGREE


def is_locked(person, lock_center_x):
    """Return (locked, error_x, error_y) in pixels.

    error_y is measured against the LiDAR's boresight row, not the image centre.
    The original computed only a horizontal condition and then discarded its own
    `locked` return value at the call site. Without a vertical check, flying at
    altitude means the forward beam shoots over the person's head while the
    camera still sees them perfectly.
    """
    if person is None or lock_center_x is None:
        return False, 0.0, 0.0

    error_x = person.center_x - lock_center_x
    error_y = person.center_y - cfg.LIDAR_BORESIGHT_ROW_PX
    locked = abs(error_x) < cfg.LIDAR_HGATE_PX
    return locked, error_x, error_y


def select(persons):
    """Return the tracked person, or None if the target was not re-found.

    None means "this frame did not contain our target" and is counted as a lost
    frame by the caller. It does not mean "no people in view".
    """
    global _target

    if not persons:
        return None

    prev = _target

    if prev is None:
        # Designation: largest box, i.e. the closest person. Only used to pick
        # the initial target at engage time; from then on identity is carried
        # by association below.
        _target = max(persons, key=lambda p: p.area)
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
        # Gate failed. Better to lose the target briefly than to lock onto the
        # wrong person; the caller's lost-frame logic handles the gap, and
        # reset() re-designates after a genuine loss.
        return None

    _target = min(candidates, key=distance_to_prev)
    return _target
