"""Two independent PID controllers: yaw from pixel error, forward from range.

Architecture unchanged. The corrections here are the yaw output sign (the
original loop was positive feedback), forward-velocity gating on alignment,
a continuous deadband, and reset discipline.
"""

import logging
import math
import time

from simple_pid import PID

import config as cfg
import drone

_log = logging.getLogger(__name__)

# Kept as module-level names for compatibility with existing call sites.
YAW_DEADBAND = cfg.YAW_DEADBAND_PX
TARGET_DISTANCE = cfg.TARGET_DISTANCE_CM
MAX_YAW_RATE = cfg.MAX_YAW_RATE_DEG_S
MAX_FORWARD_SPEED = cfg.MAX_FORWARD_SPEED_MS

# output_limits are set in the constructor rather than assigned afterwards so
# integral clamping is armed from the very first sample.
yaw_pid = PID(
    Kp=cfg.YAW_KP,
    Ki=cfg.YAW_KI,
    Kd=cfg.YAW_KD,
    setpoint=0.0,
    output_limits=(-cfg.MAX_YAW_RATE_DEG_S, cfg.MAX_YAW_RATE_DEG_S),
)

distance_pid = PID(
    Kp=cfg.DIST_KP,
    Ki=cfg.DIST_KI,
    Kd=cfg.DIST_KD,
    setpoint=cfg.TARGET_DISTANCE_CM,
    output_limits=(-cfg.MAX_FORWARD_SPEED_MS, cfg.MAX_FORWARD_SPEED_MS),
)

_last_forward = 0.0
_last_slew_time = None


def reset():
    """Clear both PIDs and the slew state.

    simple_pid computes dt as (now - self._last_time), so skipping calls leaves
    a stale timestamp. The original skipped yaw_pid inside the deadband, skipped
    distance_pid when distance was None, and skipped BOTH for the entire
    SEARCHING excursion -- up to 18 s. With Ki=Kd=0 that is harmless, which is
    why it never bit. But once Ki is non-zero, the first call after such a gap
    evaluates `_integral += Ki * error * dt` with dt ~= 18 s, which clamps
    straight to output_limits: one full-authority command at the worst possible
    moment. The derivative fails the other way -- `Kd * d_error / dt` with a
    large dt under-responds rather than spiking -- so a long gap silently
    removes the damping you tuned for.

    reset() re-bases _last_time and clears every term, so both failure modes go
    away. Called on every state entry and whenever a gate closes.
    """
    global _last_forward, _last_slew_time
    yaw_pid.reset()
    distance_pid.reset()
    _last_forward = 0.0
    _last_slew_time = None


def _deadband_error(error_x):
    """Shrink the error by the deadband instead of zeroing the output.

    The original produced yaw_rate = 0 inside 50 px and 5.1 deg/s at 51 px -- a
    5 deg/s step at the boundary, which makes the drone hunt across the edge in
    a limit cycle. Shrinking keeps the output continuous through zero while
    still providing a genuine dead zone.
    """
    if abs(error_x) <= cfg.YAW_DEADBAND_PX:
        return 0.0
    return error_x - math.copysign(cfg.YAW_DEADBAND_PX, error_x)


def _slew(desired, now):
    """Rate-limit increases in commanded forward speed.

    Reductions toward zero are applied immediately: slewing a stop would delay
    a safety response, so the asymmetry is deliberate.
    """
    global _last_forward, _last_slew_time

    if _last_slew_time is None:
        _last_slew_time = now
        _last_forward = 0.0

    dt = max(0.0, now - _last_slew_time)
    _last_slew_time = now

    if abs(desired) <= abs(_last_forward):
        _last_forward = desired
        return desired

    max_step = cfg.FORWARD_SLEW_MS2 * dt
    delta = desired - _last_forward
    if abs(delta) > max_step:
        desired = _last_forward + math.copysign(max_step, delta)

    _last_forward = desired
    return desired


def update(error_x, distance_cm, yaw_only=False, send=True, now=None):
    """Compute and optionally send one setpoint. Returns a telemetry dict.

    Both PIDs are always called and their *outputs* gated -- never the calls
    themselves -- so dt stays continuous.

    `now` is injectable so the slew limiter can be tested deterministically;
    flight callers leave it as None and get the real clock.
    """
    if now is None:
        now = time.monotonic()

    # ------------------------------------------------------------ yaw ------
    # Sign derivation is documented in full at cfg.YAW_PID_OUTPUT_SIGN. In
    # short: the image is in true world orientation, so a target on the right
    # gives error_x > 0 and needs a POSITIVE yaw rate to turn toward it, but
    # simple_pid returns Kp*(setpoint - input) which is negative. Without the
    # negation the loop is positive feedback and the drone spins away from the
    # target until it leaves frame.
    yaw_rate = cfg.YAW_PID_OUTPUT_SIGN * yaw_pid(_deadband_error(error_x))

    # ------------------------------------------------------- forward -------
    bearing_deg = abs(error_x) / cfg.PIXELS_PER_DEGREE
    raw_pid_out = distance_pid(
        distance_cm if distance_cm is not None else cfg.TARGET_DISTANCE_CM
    )

    gate_reason = None
    if yaw_only:
        forward = 0.0
        gate_reason = "yaw_only"
    elif distance_cm is None:
        # No trustworthy range: stale, out of gate, or sensor unhealthy.
        forward = 0.0
        gate_reason = "no_distance"
    elif bearing_deg > cfg.FORWARD_ALIGN_LIMIT_DEG:
        # The range that would authorise forward motion was measured along a
        # boresight the drone is no longer pointing down, and with yaw-only
        # control forward motion off-axis is displacement in an arbitrary
        # direction. Without this gate the drone flies forward on a frozen
        # range while spinning -- a spiral with no timeout to stop it.
        forward = 0.0
        gate_reason = "misaligned"
    else:
        forward = -raw_pid_out
        # Project onto the line of sight. Physically motivated, and it removes
        # the step discontinuity at the gate boundary that would cause a lurch.
        forward *= max(0.0, math.cos(math.radians(bearing_deg)))

    if gate_reason is not None:
        # Do not let the integrator wind up while the output is gated off.
        distance_pid.reset()

    # Hard floor: anything this close is either the person or an obstacle, and
    # both demand the same response. This is the only obstacle protection
    # available, since the LiDAR is committed to ranging the target.
    backing_off = False
    if distance_cm is not None and distance_cm < cfg.MIN_SAFE_DISTANCE_CM:
        forward = min(forward, -cfg.BACKOFF_SPEED_MS)
        backing_off = True

    forward = _slew(forward, now)

    telemetry = {
        "error_x": error_x,
        "bearing_deg": bearing_deg,
        "distance_cm": distance_cm,
        "yaw_rate": yaw_rate,
        "forward": forward,
        "gate": gate_reason or "open",
        "backing_off": backing_off,
    }

    if send:
        drone.move(forward_speed=forward, yaw_rate=yaw_rate)

    return telemetry


def hold(send=True):
    """Zero output with the controller left in a clean state."""
    reset()
    if send:
        drone.move(0.0, 0.0, 0.0, 0.0)
    return {
        "error_x": 0.0,
        "bearing_deg": 0.0,
        "distance_cm": None,
        "yaw_rate": 0.0,
        "forward": 0.0,
        "gate": "hold",
        "backing_off": False,
    }
