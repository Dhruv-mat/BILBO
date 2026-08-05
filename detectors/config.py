"""Central configuration for BILBO.

Every safety-critical constant lives here so there is exactly one source of
truth. This exists because the pre-review code defined MAX_YAW_RATE twice with
different values, and derived the image width in main.py while hardcoding it in
tracker.py -- both of which are correctness defects, not style issues.

Units are named in every constant. Mixed units caused a real 100x bug in the
LiDAR parallax correction (metres divided by centimetres), so the suffix is
mandatory: _CM, _M, _S, _PX, _DEG.
"""

import os

# Ground testing lives in bench.py, which is a separate program. There is
# deliberately no gate-bypass flag in the flight configuration: the engagement
# gates (armed, minimum altitude, confirmed target) have no off switch.

# ---------------------------------------------------------------- devices ----

# MAVLink runs over the FTDI adapter. Use a /dev/serial/by-id/ path, never
# /dev/ttyUSB0: if a second USB serial device is ever present, kernel
# enumeration order can swap them between boots and the drone would attempt
# MAVLink against the LiDAR. Read the exact string off the hardware with:
#     ls -l /dev/serial/by-id/
MAVLINK_DEVICE = "/dev/serial/by-id/SET-ME-SEE-COMMENT-ABOVE"

# If MAVLINK_DEVICE does not exist, drone.connect() falls back to globbing
# by-id for these patterns and requires EXACTLY ONE match. One match is
# unambiguous, so the anti-swap guarantee is preserved; zero or several is a
# hard failure rather than a guess.
MAVLINK_DEVICE_GLOBS = (
    "/dev/serial/by-id/*FTDI*",
    "/dev/serial/by-id/*FT232*",
)

# Must match the Pixhawk's SERIALn_BAUD for the port the FTDI is wired to.
# 57600 is adequate now that setpoints are rate limited; 921600 gives latency
# headroom if you are willing to change the Pixhawk parameter.
MAVLINK_BAUD = 57600

# The TF Luna has its own dedicated UART. This must NOT be the same device as
# MAVLINK_DEVICE -- sharing one port was the defect that prevented startup.
LIDAR_DEVICE = "/dev/ttyAMA0"
LIDAR_BAUD = 115200

LED_DEVICE = "/dev/spidev0.0"
LED_COUNT = 25
LED_BRIGHTNESS = 0.3
LED_SPI_KHZ = 800

# --------------------------------------------------------------- logging ----

LOG_DIR = os.path.expanduser("~/bilbo-logs")
LOG_LEVEL = "INFO"
LOG_MAX_BYTES = 8 * 1024 * 1024
LOG_BACKUPS = 5
# Human-readable status lines are throttled to this rate. The full-rate record
# goes to the CSV, which is the only thing that can diagnose an anomaly after
# the fact.
STATUS_LOG_INTERVAL_S = 0.5
CSV_FLUSH_INTERVAL_S = 1.0

# Console logging and the camera preview both cost CPU and add pipeline jitter,
# and nobody watches a preview in flight. Bench only, env-driven so they cannot
# persist into a flight:
#     BILBO_DEBUG_CONSOLE=1   mirror the log to stderr
#     BILBO_DEBUG_PREVIEW=1   camera preview window
#     BILBO_DEBUG_DRAW=1      bounding-box overlay
#
# Console logging is off by default because writing to stdout/stderr at loop rate
# is blocking I/O: if the consumer stops draining the pipe the write blocks and
# the flight loop freezes. `tail -f ~/bilbo-logs/bilbo.log` in another terminal
# has none of that risk and is the better way to watch a bench run.
DEBUG_CONSOLE = os.environ.get("BILBO_DEBUG_CONSOLE", "0") == "1"
DEBUG_PREVIEW = os.environ.get("BILBO_DEBUG_PREVIEW", "0") == "1"
DEBUG_DRAW = os.environ.get("BILBO_DEBUG_DRAW", "0") == "1"

# ---------------------------------------------------------------- timing ----

TICK_HZ = 15.0
TICK_PERIOD_S = 1.0 / TICK_HZ

# Floor on the interval between MAVLink setpoints. Prevents a fast loop from
# saturating the serial TX buffer, which makes the Pixhawk act on setpoints
# that are seconds stale. Must stay well inside ArduPilot's guided timeout.
MIN_SETPOINT_INTERVAL_S = 1.0 / 20.0

# A tick that overruns by more than this means something blocked. Escalates to
# EMERGENCY rather than silently flying on a frozen loop.
TICK_OVERRUN_LIMIT_S = 0.5
MAX_CONSECUTIVE_FAULTS = 5

# Heartbeat is emitted FROM THE CONTROL LOOP, never a thread: a threaded
# heartbeat would keep reassuring the Pixhawk that the companion is healthy
# while the flight loop was hung, which is actively harmful. Loop liveness and
# heartbeat liveness must be the same thing.
HEARTBEAT_TX_INTERVAL_S = 1.0

# No HEARTBEAT from the Pixhawk for this long means the link is gone. Without
# this check the code keeps computing PID outputs and sending setpoints into a
# dead port while reading a cached "GUIDED" forever.
LINK_TIMEOUT_S = 2.0
RECONNECT_INTERVAL_S = 1.0

# RC data older than this is treated as OFF. Fail closed.
RC_STALE_S = 1.0

# Detections older than this mean the camera pipeline stalled. Distinct from
# "no person detected".
VISION_MAX_AGE_S = 0.3
MAX_VISION_FAULTS = 30  # ~2 s at 15 Hz before escalating

# -------------------------------------------------------------- geometry ----

# The camera stream size. Asserted against the real configured size during
# preflight; a mismatch is a hard failure rather than a silent divergence
# between two hardcoded values.
IMAGE_WIDTH_PX = 640
IMAGE_HEIGHT_PX = 480
CAMERA_FOV_DEG = 78.0
PIXELS_PER_DEGREE = IMAGE_WIDTH_PX / CAMERA_FOV_DEG  # ~8.2 px/deg

# The camera is mounted INVERTED and camera.py applies Transform(hflip, vflip)
# to cancel that. Two mirrors compose to a pure 180 deg rotation, so the
# delivered image is in TRUE world orientation: a person physically to the
# drone's right appears on the right of the image, and image y increases
# downward as normal. The yaw sign chain in controller.py depends on this.
CAMERA_IS_INVERTED = True

# ----------------------------------------------------------------- lidar ----

LIDAR_MIN_CM = 20
LIDAR_MAX_CM = 800
# The TF Luna reports signal strength in bytes 4-5. Low strength means the
# reading is meaningless -- the dominant outdoor failure mode is bright sun on
# dark clothing. 65535 indicates saturation.
LIDAR_MIN_STRENGTH = 100
LIDAR_RX_BUFFER_CAP = 512  # bounded so a runaway sensor cannot grow memory

# A retained distance older than this is discarded, which makes the controller
# command zero forward velocity instead of flying on ancient data.
LIDAR_MAX_AGE_S = 1.0
# The ~2 deg beam slips past a person onto the ground or sky behind them,
# stepping e.g. 200 -> 1500 cm. A step this large must repeat before it is
# believed, which absorbs single-frame slips without ignoring genuine motion.
LIDAR_MAX_JUMP_CM = 150
LIDAR_JUMP_CONFIRMS = 2
LIDAR_HEALTH_FRAMES = 5  # validated frames required at preflight

# Horizontal gate for trusting a range reading. The beam is ~2 deg wide, so
# this must be much tighter than YAW_DEADBAND_PX (~6.1 deg) -- a target 6 deg
# off-axis is entirely outside the beam and the range would be of whatever is
# behind them.
LIDAR_HGATE_PX = 15

# Vertical gate. Without this, flying at altitude means the forward beam shoots
# over the person's head while the camera still sees them fine.
# CALIBRATE LIDAR_BORESIGHT_ROW_PX EMPIRICALLY: aim the beam at a known target
# and note which image row it lands on. The default is only a starting guess.
LIDAR_BORESIGHT_ROW_PX = IMAGE_HEIGHT_PX // 2
LIDAR_VGATE_PX = 40

# Horizontal offset from camera optical axis to LiDAR, in CENTIMETRES.
# The pre-review code divided 0.05 m by a centimetre distance, making the
# parallax correction 100x too small.
LIDAR_BASELINE_CM = 5.0
# +1 if the LiDAR sits to the right of the camera in the delivered image,
# -1 if to the left. VERIFY THIS PHYSICALLY -- it was previously asserted
# without justification.
LIDAR_OFFSET_SIGN = 1.0

# --------------------------------------------------------------- tracker ----

PERSON_CLASS = 0  # COCO index
# 0.5 is low for a flight-critical gate.
CONF_THRESHOLD = 0.65
MIN_BOX_AREA_PX = 1200

# Nearest-neighbour association gate. A detection must be within this many
# pixels of the previous target's centre and within TRACK_AREA_RATIO of its
# area to be accepted as the same person. This is what stops a passer-by
# walking between the drone and the target from stealing the lock.
TRACK_GATE_PX = 140.0
TRACK_AREA_RATIO = 2.5

# ------------------------------------------------------------ controller ----

YAW_DEADBAND_PX = 50.0

# The yaw PID output is NEGATED. Derivation, verified against the physical
# build and confirmed by tools/verify_yaw_sign.py:
#   1. MAVLink yaw_rate is rad/s about the NED *down* axis, so positive =
#      clockwise viewed from above = nose right.
#   2. Camera is mounted inverted and Transform(hflip, vflip) cancels it, so
#      the image is in true world orientation: person physically right =>
#      center_x large => error_x > 0.
#   3. To turn toward a target on the right we need yaw_rate POSITIVE.
#   4. simple_pid returns Kp*(setpoint - input) = -Kp*error_x, i.e. NEGATIVE
#      when error_x > 0 -- it would turn the nose LEFT, away from the target.
#   5. Therefore the output must be negated. Without this the loop is positive
#      feedback and the drone spins away from the target until it exits frame.
# Flip to +1.0 ONLY if the bench tool disagrees, and re-verify the chain above.
YAW_PID_OUTPUT_SIGN = -1.0

YAW_KP = 0.10
YAW_KI = 0.0
# Ki stays 0 for first flight: a steady crosswind leaves a fixed bearing lag,
# which is acceptable and carries no windup risk. Kd is now SAFE to tune
# (controller.reset() is called on every gate close and state entry, so a
# skipped-call dt spike can no longer produce a full-authority command), but
# shipping an untuned derivative on noisy pixel error would amplify detector
# jitter. Tune it on the bench, not in the air.
YAW_KD = 0.0

# 15 deg/s could not keep up with a person walking laterally at 1.5 m/s at 5 m
# (~17 deg/s of bearing rate), so the target left the 78 deg FOV. This value is
# only safe because YAW_PID_OUTPUT_SIGN above is correct -- raising authority on
# an inverted loop makes a runaway faster.
MAX_YAW_RATE_DEG_S = 50.0

TARGET_DISTANCE_CM = 400.0
DIST_KP = 0.006  # saturates near 165 cm of error
DIST_KI = 0.0
DIST_KD = 0.0
MAX_FORWARD_SPEED_MS = 1.0
# Rate limit on commanded forward velocity so no single bad range reading can
# produce a velocity step.
FORWARD_SLEW_MS2 = 0.75

# Forward velocity is gated off beyond this bearing error. Two reasons: the
# range that authorises forward motion was measured along a boresight the drone
# is no longer pointing down, and with yaw-only control forward motion only
# closes distance when the nose is roughly on the target. Without this gate an
# off-axis target makes the drone fly forward on a frozen range while spinning.
FORWARD_ALIGN_LIMIT_DEG = 15.0

# Anything this close is either the person or an obstacle; both demand the same
# response. This is the only obstacle protection available, since the LiDAR is
# committed to ranging the target.
MIN_SAFE_DISTANCE_CM = 150.0
BACKOFF_SPEED_MS = 0.4

# Final clamp applied inside drone.move(), independent of the PID output_limits,
# so no upstream bug can emit an absurd velocity.
HARD_MAX_SPEED_MS = 2.0
HARD_MAX_YAW_RATE_DEG_S = 60.0

# --------------------------------------------------- state machine / RC -----

CH_ENABLE = 8
# 3-position: OFF / YAW_ONLY / FULL. YAW_ONLY validates the yaw loop in the air
# with zero translation risk, which matters because the yaw sign was inverted.
SWITCH_MID_LOW_US = 1300
SWITCH_MID_HIGH_US = 1700

# Time-based, not frame-based: 6 frames at ~30 fps was 0.2 s, far too short for
# a person turning away or briefly occluded, and the meaning silently changed
# with frame rate.
LOST_TRACK_S = 1.2
# Re-acquisition needs confirmation. A single frame re-locking meant one false
# positive could flip-flop SEARCHING<->TRACKING forever, resetting the search
# timer each time so RTL never fired.
REACQUIRE_FRAMES = 3
TARGET_CONFIRM_FRAMES = 3

SEARCH_YAW_RATE_DEG_S = 35.0
# 8 deg/s for 10 s swept only 80 deg -- barely one FOV, so the search almost
# always timed out having seen nothing. 35 deg/s for 18 s is ~1.7 revolutions.
SEARCH_TIMEOUT_S = 18.0

# Cumulative watchdog, independent of the search timer and NOT resettable by
# transient re-acquisition. This is what actually guarantees RTL fires.
MAX_LOST_TRACK_S = 25.0
# Absolute ceiling on continuous tracking. Pi-side substitute for the excluded
# geofence, and strictly weaker -- any Pi bug can defeat it.
MAX_TRACK_DURATION_S = 240.0

MIN_TRACK_ALT_M = 2.0
EMERGENCY_HOLD_S = 3.0
MAX_RTL_ATTEMPTS = 2
