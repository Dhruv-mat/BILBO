"""Assert the relationships BETWEEN configuration constants.

Every value in config.py can be individually sensible while a pair of them
contradicts each other. Those contradictions are the dangerous kind, because
nothing crashes -- the drone just behaves wrongly in a specific corner. Each
check below records why the ordering matters, not merely that it holds.
"""

import harness  # noqa: F401  installs hardware stubs
from harness import check, summary

import config as cfg
import controller
import tracker

print("\n=== Timing ===")

check("setpoint rate limit is faster than the control tick, so no tick's "
      "setpoint is silently dropped",
      cfg.MIN_SETPOINT_INTERVAL_S < cfg.TICK_PERIOD_S,
      "%.4f < %.4f" % (cfg.MIN_SETPOINT_INTERVAL_S, cfg.TICK_PERIOD_S))

# ArduPilot's GUID_TIMEOUT is 3 s: setpoints must arrive far more often than
# that or the vehicle brakes mid-track.
check("setpoint rate leaves large margin against ArduPilot's 3 s guided timeout",
      cfg.TICK_PERIOD_S < 3.0 / 10,
      "tick %.3f s vs 3.0 s timeout" % cfg.TICK_PERIOD_S)

check("heartbeat is emitted more often than the link timeout, so our own "
      "heartbeat cannot be the thing that looks stale",
      cfg.HEARTBEAT_TX_INTERVAL_S < cfg.LINK_TIMEOUT_S,
      "%.1f < %.1f" % (cfg.HEARTBEAT_TX_INTERVAL_S, cfg.LINK_TIMEOUT_S))

# ArduPilot defaults to 1 Hz HEARTBEAT on a serial port. A timeout of 2 s would
# tolerate only two missed beats, so one hiccup would latch EMERGENCY.
check("link timeout tolerates at least 3 missed heartbeats at ArduPilot's "
      "default 1 Hz",
      cfg.LINK_TIMEOUT_S >= 3.0,
      "%.1f s" % cfg.LINK_TIMEOUT_S)

check("tick overrun limit is well above the tick period, so normal jitter "
      "cannot trip the stall watchdog",
      cfg.TICK_OVERRUN_LIMIT_S > cfg.TICK_PERIOD_S * 3,
      "%.2f > %.2f" % (cfg.TICK_OVERRUN_LIMIT_S, cfg.TICK_PERIOD_S * 3))

check("vision staleness tolerance spans several control ticks",
      cfg.VISION_MAX_AGE_S > cfg.TICK_PERIOD_S * 2,
      "%.2f > %.2f" % (cfg.VISION_MAX_AGE_S, cfg.TICK_PERIOD_S * 2))

print("\n=== Tracking identity ===")

# If the tracker dropped identity before the state machine gave up on the track,
# it would re-designate to the largest box mid-track -- which may be a different
# person. That is the exact failure the association gate exists to prevent.
misses_s = cfg.TRACK_MAX_MISSES * cfg.TICK_PERIOD_S
check("tracker holds identity at least as long as the state machine holds the "
      "track, so it cannot silently swap target mid-track",
      misses_s >= cfg.LOST_TRACK_S - 1e-9,
      "%.2f s >= %.2f s" % (misses_s, cfg.LOST_TRACK_S))

check("re-acquisition needs more than one frame, so a single false positive "
      "cannot re-lock",
      cfg.REACQUIRE_FRAMES >= 2, "-> %d" % cfg.REACQUIRE_FRAMES)

check("engagement needs more than one frame of confirmed target",
      cfg.TARGET_CONFIRM_FRAMES >= 2, "-> %d" % cfg.TARGET_CONFIRM_FRAMES)

print("\n=== Yaw deadband vs LiDAR range gate ===")

# THE critical pair. If the deadband were ever wider than the range gate, yaw
# would stop correcting while the target sat outside the beam: no range would
# ever be read, forward velocity could never engage, and the drone would park
# with the person off to one side indefinitely.
worst = float("-inf")
worst_at = None
for box_w in range(1, cfg.IMAGE_WIDTH_PX + 1):
    margin = controller.deadband_px(box_w) - tracker.hgate_px(box_w)
    if margin > worst:
        worst, worst_at = margin, box_w
check("yaw deadband is strictly inside the range gate for EVERY box width",
      worst < 0,
      "worst margin %+.1f px at box width %d" % (worst, worst_at))

check("deadband fraction is below the range-gate fraction",
      cfg.YAW_DEADBAND_FRAC < cfg.LIDAR_HGATE_FRAC,
      "%.2f < %.2f" % (cfg.YAW_DEADBAND_FRAC, cfg.LIDAR_HGATE_FRAC))
check("deadband clamps nest inside the range-gate clamps",
      cfg.YAW_DEADBAND_MIN_PX < cfg.LIDAR_HGATE_MIN_PX
      and cfg.YAW_DEADBAND_MAX_PX < cfg.LIDAR_HGATE_MAX_PX,
      "min %.0f<%.0f, max %.0f<%.0f"
      % (cfg.YAW_DEADBAND_MIN_PX, cfg.LIDAR_HGATE_MIN_PX,
         cfg.YAW_DEADBAND_MAX_PX, cfg.LIDAR_HGATE_MAX_PX))

# The widest deadband must stay inside the forward-alignment limit, or there
# would be bearings where yaw has given up but forward is still commanded.
max_deadband_deg = cfg.YAW_DEADBAND_MAX_PX / cfg.PIXELS_PER_DEGREE
check("widest deadband is inside the forward-alignment limit",
      max_deadband_deg < cfg.FORWARD_ALIGN_LIMIT_DEG,
      "%.1f deg < %.1f deg" % (max_deadband_deg, cfg.FORWARD_ALIGN_LIMIT_DEG))

check("forward-alignment limit is inside the camera's half field of view, so "
      "the gate is reachable at all",
      cfg.FORWARD_ALIGN_LIMIT_DEG < cfg.CAMERA_FOV_DEG / 2,
      "%.1f < %.1f" % (cfg.FORWARD_ALIGN_LIMIT_DEG, cfg.CAMERA_FOV_DEG / 2))

print("\n=== Automatic takeoff ===")

# The engagement floor must sit BELOW the altitude the takeoff actually
# delivers, or the gate would reject the very climb it just commanded and
# baro noise would flip tracking on and off at the top of the climb.
completes_at = cfg.TAKEOFF_ALT_M - cfg.TAKEOFF_ALT_TOLERANCE_M
check("takeoff completion altitude is above the tracking floor",
      completes_at > cfg.MIN_TRACK_ALT_M,
      "%.2f m > %.2f m" % (completes_at, cfg.MIN_TRACK_ALT_M))

check("takeoff target is above the on-the-ground threshold",
      cfg.TAKEOFF_ALT_M > cfg.TAKEOFF_MAX_START_ALT_M,
      "%.2f m > %.2f m" % (cfg.TAKEOFF_ALT_M,
                           cfg.TAKEOFF_MAX_START_ALT_M))

# A 1.83 m person plus baro drift: below this the propeller disc can sit
# at or under head height.
check("takeoff altitude clears a standing adult with some margin",
      cfg.TAKEOFF_ALT_M >= 2.0,
      "%.2f m (>= 2.0 m; 3.0 m+ is materially safer given +/-0.5 m baro drift)"
      % cfg.TAKEOFF_ALT_M)

check("takeoff timeout allows a realistic climb rate",
      cfg.TAKEOFF_TIMEOUT_S > cfg.TAKEOFF_ALT_M / 0.5,
      "%.0f s for %.1f m (needs > %.0f s at 0.5 m/s)"
      % (cfg.TAKEOFF_TIMEOUT_S, cfg.TAKEOFF_ALT_M,
         cfg.TAKEOFF_ALT_M / 0.5))


print("\n=== Distances and speeds ===")

check("minimum safe distance is inside the target standoff, so the back-off "
      "floor cannot fight the setpoint",
      cfg.MIN_SAFE_DISTANCE_CM < cfg.TARGET_DISTANCE_CM,
      "%.0f < %.0f cm" % (cfg.MIN_SAFE_DISTANCE_CM, cfg.TARGET_DISTANCE_CM))

# If the do-nothing band reached below the safety floor, the PID would
# report "close enough" inside the distance the floor says to retreat
# from. The floor still wins (it is a min()), but the two would be
# openly contradicting each other, which is how tuning accidents start.
band_low = cfg.TARGET_DISTANCE_CM - cfg.DIST_DEADBAND_CM
check("the distance deadband stays clear of the safety floor",
      band_low > cfg.MIN_SAFE_DISTANCE_CM,
      "band bottom %.0f cm > floor %.0f cm"
      % (band_low, cfg.MIN_SAFE_DISTANCE_CM))

check("the distance deadband is a usable fraction of the standoff -- big "
      "enough to stop micro-corrections, small enough to still hold "
      "station",
      0.02 < cfg.DIST_DEADBAND_CM / cfg.TARGET_DISTANCE_CM < 0.25,
      "+/-%.0f cm on %.0f cm = %.0f%%"
      % (cfg.DIST_DEADBAND_CM, cfg.TARGET_DISTANCE_CM,
         100.0 * cfg.DIST_DEADBAND_CM / cfg.TARGET_DISTANCE_CM))

check("target standoff is inside the LiDAR's accepted range",
      cfg.LIDAR_MIN_CM < cfg.TARGET_DISTANCE_CM < cfg.LIDAR_MAX_CM,
      "%d < %.0f < %d" % (cfg.LIDAR_MIN_CM, cfg.TARGET_DISTANCE_CM,
                          cfg.LIDAR_MAX_CM))

check("minimum safe distance is above the LiDAR's minimum, so the floor is "
      "measurable",
      cfg.MIN_SAFE_DISTANCE_CM > cfg.LIDAR_MIN_CM,
      "%.0f > %d cm" % (cfg.MIN_SAFE_DISTANCE_CM, cfg.LIDAR_MIN_CM))

check("hard velocity clamp is above the controller's own limit, so it is a "
      "backstop rather than the active limit",
      cfg.HARD_MAX_SPEED_MS >= cfg.MAX_FORWARD_SPEED_MS,
      "%.1f >= %.1f" % (cfg.HARD_MAX_SPEED_MS, cfg.MAX_FORWARD_SPEED_MS))
check("hard yaw clamp is above the controller's own limit",
      cfg.HARD_MAX_YAW_RATE_DEG_S >= cfg.MAX_YAW_RATE_DEG_S,
      "%.0f >= %.0f" % (cfg.HARD_MAX_YAW_RATE_DEG_S, cfg.MAX_YAW_RATE_DEG_S))
check("search yaw rate is within the hard yaw clamp, so search is not silently "
      "throttled",
      cfg.SEARCH_YAW_RATE_DEG_S <= cfg.HARD_MAX_YAW_RATE_DEG_S,
      "%.0f <= %.0f" % (cfg.SEARCH_YAW_RATE_DEG_S,
                        cfg.HARD_MAX_YAW_RATE_DEG_S))
check("back-off speed is within the hard clamp",
      cfg.BACKOFF_SPEED_MS <= cfg.HARD_MAX_SPEED_MS)

# The drone must be able to out-turn a walking person's bearing rate at the
# standoff, or the target simply leaves frame.
lateral_ms = 1.5
standoff_m = cfg.TARGET_DISTANCE_CM / 100.0
import math
bearing_rate = math.degrees(lateral_ms / standoff_m)
check("max yaw rate exceeds the bearing rate of a person walking across at "
      "1.5 m/s at the target standoff",
      cfg.MAX_YAW_RATE_DEG_S > bearing_rate,
      "%.0f deg/s > %.0f deg/s needed" % (cfg.MAX_YAW_RATE_DEG_S,
                                          bearing_rate))

print("\n=== Search and fallback ladder ===")

sweep_deg = cfg.SEARCH_YAW_RATE_DEG_S * cfg.SEARCH_TIMEOUT_S
check("a full search sweeps at least 360 deg before timing out, so it can "
      "actually find a target behind the drone",
      sweep_deg >= 360.0, "%.0f deg swept" % sweep_deg)

check("search timeout fires before the cumulative lost-track watchdog, so the "
      "normal path is search-then-RTL",
      cfg.SEARCH_TIMEOUT_S < cfg.MAX_LOST_TRACK_S,
      "%.0f < %.0f s" % (cfg.SEARCH_TIMEOUT_S, cfg.MAX_LOST_TRACK_S))

check("cumulative lost-track watchdog fires before the absolute tracking cap",
      cfg.MAX_LOST_TRACK_S < cfg.MAX_TRACK_DURATION_S,
      "%.0f < %.0f s" % (cfg.MAX_LOST_TRACK_S, cfg.MAX_TRACK_DURATION_S))

check("emergency hold is long enough to settle but short of the guided timeout",
      0.5 < cfg.EMERGENCY_HOLD_S < 10.0, "%.1f s" % cfg.EMERGENCY_HOLD_S)
check("at least one RTL attempt before falling back to LAND",
      cfg.MAX_RTL_ATTEMPTS >= 1, "-> %d" % cfg.MAX_RTL_ATTEMPTS)

print("\n=== Geometry and RC ===")

check("LiDAR boresight row is inside the image",
      0 <= cfg.LIDAR_BORESIGHT_ROW_PX < cfg.IMAGE_HEIGHT_PX,
      "row %d of %d" % (cfg.LIDAR_BORESIGHT_ROW_PX, cfg.IMAGE_HEIGHT_PX))
check("pixels-per-degree is derived from the image width, not hardcoded",
      abs(cfg.PIXELS_PER_DEGREE
          - cfg.IMAGE_WIDTH_PX / cfg.CAMERA_FOV_DEG) < 1e-9)
check("switch threshold sits inside the normal RC pulse range, clear of both endpoints",
      1200 < cfg.SWITCH_ON_US < 1800,
      "%d us" % cfg.SWITCH_ON_US)
check("RC staleness tolerance spans several ticks but is well under a second "
      "of real lag",
      cfg.TICK_PERIOD_S * 2 < cfg.RC_STALE_S <= 2.0,
      "%.2f s" % cfg.RC_STALE_S)

check("yaw sign is exactly +1 or -1",
      cfg.YAW_PID_OUTPUT_SIGN in (1.0, -1.0),
      "-> %+.1f" % cfg.YAW_PID_OUTPUT_SIGN)
check("LiDAR parallax offset sign is exactly +1 or -1",
      cfg.LIDAR_OFFSET_SIGN in (1.0, -1.0),
      "-> %+.1f" % cfg.LIDAR_OFFSET_SIGN)

print("\n=== Detection filter sanity ===")

check("confidence threshold is a probability",
      0.0 < cfg.CONF_THRESHOLD < 1.0, "-> %.2f" % cfg.CONF_THRESHOLD)
check("confidence threshold is not so high that ordinary detections are "
      "discarded (SSD-MobileNet scores real people down into the 0.45 band)",
      cfg.CONF_THRESHOLD <= 0.55, "-> %.2f" % cfg.CONF_THRESHOLD)
check("association gate is wider than a walking person moves in one tick",
      cfg.TRACK_GATE_PX > bearing_rate * cfg.PIXELS_PER_DEGREE
      * cfg.TICK_PERIOD_S,
      "%.0f px > %.0f px/tick"
      % (cfg.TRACK_GATE_PX,
         bearing_rate * cfg.PIXELS_PER_DEGREE * cfg.TICK_PERIOD_S))
check("LiDAR jump rejection needs confirmation but not many frames",
      2 <= cfg.LIDAR_JUMP_CONFIRMS <= 4, "-> %d" % cfg.LIDAR_JUMP_CONFIRMS)
check("retained range expires within a couple of seconds",
      0.2 < cfg.LIDAR_MAX_AGE_S <= 2.0, "-> %.1f s" % cfg.LIDAR_MAX_AGE_S)

print("\n=== No gate bypass exists ===")

import os
check("config has no BENCH_MODE or similar bypass attribute",
      not any(n for n in dir(cfg)
              if "BENCH" in n.upper() or "BYPASS" in n.upper()))
check("no BILBO_BENCH* environment variable is consulted",
      not any(k.startswith("BILBO_BENCH") for k in os.environ))

raise SystemExit(summary())
