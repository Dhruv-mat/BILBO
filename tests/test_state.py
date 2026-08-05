"""Drive the real main.py state machine with a fake Pixhawk and camera.

Targets every transition defect identified in the review.
"""

import os
import sys
import types

import harness  # noqa: F401  installs the hardware stubs
from harness import check, summary, FakePerson

import config as cfg

# ---- fake drone, installed BEFORE main/controller import it ---------------

sent = []           # every setpoint that reached "the wire"
mode_requests = []


class FakeDrone(types.ModuleType):
    def __init__(self):
        super().__init__("drone")
        self.reset()

    def reset(self):
        self.mode = "GUIDED"
        self.armed = True
        self.alt = 5.0
        self.rc = {cfg.CH_ENABLE: 1000}
        self._link_age = 0.0
        self._rc_age = 0.0
        self.set_mode_ok = True
        self.connected = True
        del sent[:]
        del mode_requests[:]

    # --- link
    def poll(self, max_msgs=200):
        return 0

    def send_heartbeat(self):
        return True

    def link_age(self):
        return self._link_age

    def link_ok(self):
        return self._link_age <= cfg.LINK_TIMEOUT_S

    def rc_age(self):
        return self._rc_age

    def reconnect(self):
        return False

    def is_connected(self):
        return self.connected

    def disconnect_drone(self):
        self.connected = False

    # --- cache reads
    def get_mode(self):
        return self.mode

    def is_armed(self):
        return self.armed

    def get_relative_alt(self):
        return self.alt

    def get_channel(self, ch):
        return self.rc.get(ch)

    # --- commands
    def set_mode(self, mode, timeout=1.5, retries=1):
        mode_requests.append(mode)
        if self.set_mode_ok:
            self.mode = mode
            return True
        return False

    def move(self, forward_speed=0.0, right_speed=0.0, down_speed=0.0,
             yaw_rate=0.0, force=False):
        sent.append((forward_speed, right_speed, down_speed, yaw_rate))
        return True

    def hover(self, force=False):
        return self.move(force=force)

    def stop(self, force=True):
        return self.hover(force=force)


fake_drone = FakeDrone()
sys.modules["drone"] = fake_drone


# ---- fake camera ---------------------------------------------------------

class FakeCamera(types.ModuleType):
    def __init__(self):
        super().__init__("camera")
        self.ts = 0.0
        self.persons = []

    def get_latest(self):
        return (self.ts, self.persons)

    def frame_age(self):
        return 0.0

    def inference_alive(self):
        return True

    def health(self):
        return {}

    def stop(self):
        pass


fake_camera = FakeCamera()
sys.modules["camera"] = fake_camera

import main            # noqa: E402
import tracker         # noqa: E402
import controller      # noqa: E402
from state import DroneState  # noqa: E402

tracker.configure(cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)

CENTRED = FakePerson(cfg.IMAGE_WIDTH_PX / 2, cfg.LIDAR_BORESIGHT_ROW_PX, 40000)
CLOCK = [1000.0]


def tick(n=1, persons=None, dt=None):
    """Advance the simulated clock and run n ticks."""
    dt = cfg.TICK_PERIOD_S if dt is None else dt
    for _ in range(n):
        CLOCK[0] += dt
        fake_camera.ts = CLOCK[0]
        if persons is not None:
            fake_camera.persons = persons
        main.run_tick(CLOCK[0], 0.001)
    return main.state


def fresh(state=DroneState.IDLE, enable_us=1000, persons=None):
    """Reset all machine state for an independent scenario."""
    fake_drone.reset()
    fake_drone.rc[cfg.CH_ENABLE] = enable_us
    main.state = state
    main.prev_enable = None
    main.engage_armed = False
    main.confirm_count = 0
    main.lost_since = None
    main.reacquire_count = 0
    main.last_error_x = 0.0
    main.last_confident_track = None
    main.track_started = None
    main.search_start = None
    main.search_dir = 1.0
    main.last_dist = None
    main.last_dist_t = None
    main.jump_count = 0
    main.vision_faults = 0
    main.self_commanded_mode = None
    main.rtl_requested = False
    main.emergency_since = None
    main.rtl_attempts = 0
    main.land_attempted = False
    main.fault_latch = None
    tracker.reset()
    controller.reset()
    fake_camera.persons = [] if persons is None else persons
    CLOCK[0] += 10.0


print("\n=== State machine: engagement gates ===")

fresh(DroneState.IDLE)
tick(1)
check("IDLE -> READY in GUIDED", main.state == DroneState.READY,
      "-> %s" % main.state.name)

# Switch already high at boot must NOT engage: no low->high edge was observed.
fresh(DroneState.IDLE, enable_us=1900, persons=[CENTRED])
tick(10)
check("switch high at boot cannot auto-engage",
      main.state == DroneState.READY, "-> %s" % main.state.name)

# Cycle low then high -> engages.
fresh(DroneState.READY, enable_us=1000, persons=[CENTRED])
tick(2)
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(cfg.TARGET_CONFIRM_FRAMES + 2)
check("low->high edge with armed+altitude+target engages TRACKING",
      main.state == DroneState.TRACKING, "-> %s" % main.state.name)

# Not armed -> refuse.
fresh(DroneState.READY, enable_us=1000, persons=[CENTRED])
fake_drone.armed = False
tick(2)
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(10)
check("refuses to engage while disarmed", main.state == DroneState.READY,
      "-> %s" % main.state.name)

# Too low -> refuse.
fresh(DroneState.READY, enable_us=1000, persons=[CENTRED])
fake_drone.alt = 0.4
tick(2)
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(10)
check("refuses to engage below MIN_TRACK_ALT", main.state == DroneState.READY,
      "-> %s" % main.state.name)

# No target -> refuse.
fresh(DroneState.READY, enable_us=1000, persons=[])
tick(2)
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(10)
check("refuses to engage with no confirmed target",
      main.state == DroneState.READY, "-> %s" % main.state.name)

# Stale RC reads as OFF.
fresh(DroneState.READY, enable_us=1900, persons=[CENTRED])
fake_drone._rc_age = cfg.RC_STALE_S + 1.0
tick(10)
check("stale RC is treated as OFF", main.state == DroneState.READY,
      "-> %s" % main.state.name)


print("\n=== State machine: disengagement (the one-way latch bug) ===")

fresh(DroneState.READY, enable_us=1000, persons=[CENTRED])
tick(2)
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(cfg.TARGET_CONFIRM_FRAMES + 2)
assert main.state == DroneState.TRACKING
fake_drone.rc[cfg.CH_ENABLE] = 1000
tick(1)
check("flipping the switch OFF stops tracking",
      main.state == DroneState.READY, "-> %s" % main.state.name)

# And re-engagement requires a NEW edge, not just a high switch.
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(cfg.TARGET_CONFIRM_FRAMES + 2)
check("re-engagement after disable works via a fresh edge",
      main.state == DroneState.TRACKING, "-> %s" % main.state.name)


print("\n=== State machine: loss, search, and the RTL watchdog ===")

fresh(DroneState.TRACKING, enable_us=1900, persons=[CENTRED])
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
tick(3, persons=[CENTRED])
check("tracking a visible target stays in TRACKING",
      main.state == DroneState.TRACKING, "-> %s" % main.state.name)

tick(int(cfg.LOST_TRACK_S / cfg.TICK_PERIOD_S) + 3, persons=[])
check("sustained target loss -> SEARCHING",
      main.state == DroneState.SEARCHING, "-> %s" % main.state.name)

# A SINGLE detection must not re-lock: that was the flip-flop bug.
tick(1, persons=[CENTRED])
check("one frame does not re-acquire (needs REACQUIRE_FRAMES)",
      main.state == DroneState.SEARCHING, "-> %s" % main.state.name)
tick(cfg.REACQUIRE_FRAMES, persons=[CENTRED])
check("REACQUIRE_FRAMES consecutive frames do re-acquire",
      main.state == DroneState.TRACKING, "-> %s" % main.state.name)

# Search yaw must be commanded, and toward the last-seen side.
fresh(DroneState.TRACKING, enable_us=1900)
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
main.last_error_x = +200.0     # target was to the RIGHT
tick(int(cfg.LOST_TRACK_S / cfg.TICK_PERIOD_S) + 3, persons=[])
assert main.state == DroneState.SEARCHING
del sent[:]
tick(1, persons=[])
yaws = [s[3] for s in sent]
check("SEARCHING commands yaw", any(abs(y) > 1.0 for y in yaws),
      "-> %r" % yaws)
check("search rotates toward the last-seen side (right -> +yaw)",
      all(y > 0 for y in yaws if abs(y) > 1.0), "-> %r" % yaws)
check("SEARCHING commands zero forward velocity",
      all(s[0] == 0.0 for s in sent), "-> %r" % sent)

# Search timeout -> RTL.
tick(int(cfg.SEARCH_TIMEOUT_S / cfg.TICK_PERIOD_S) + 3, persons=[])
check("search timeout -> RTL", main.state == DroneState.RTL,
      "-> %s" % main.state.name)
check("RTL was actually requested from the Pixhawk",
      "RTL" in mode_requests, "-> %r" % mode_requests)

# The cumulative watchdog must defeat a flip-flop that resets the search timer.
fresh(DroneState.TRACKING, enable_us=1900)
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
lost_ticks = int(cfg.LOST_TRACK_S / cfg.TICK_PERIOD_S) + 2
flipflops = 0
for _ in range(400):
    if main.state in (DroneState.RTL, DroneState.EMERGENCY):
        break
    tick(lost_ticks, persons=[])          # lose the target -> SEARCHING
    tick(1, persons=[CENTRED])            # one-frame false positive
    flipflops += 1
check("flip-flopping cannot postpone RTL indefinitely",
      main.state == DroneState.RTL,
      "-> %s after %d flip-flops" % (main.state.name, flipflops))


print("\n=== State machine: pilot authority ===")

fresh(DroneState.TRACKING, enable_us=1900, persons=[CENTRED])
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
fake_drone.mode = "LOITER"
del sent[:]
tick(3, persons=[CENTRED])
check("pilot mode change -> IDLE", main.state == DroneState.IDLE,
      "-> %s" % main.state.name)
check("NO setpoints are sent while the pilot has control",
      len(sent) == 0, "-> %d setpoints" % len(sent))

# Returning to GUIDED must NOT silently re-engage autonomy.
fake_drone.mode = "GUIDED"
tick(10, persons=[CENTRED])
check("returning to GUIDED does not auto-re-engage tracking",
      main.state == DroneState.READY, "-> %s" % main.state.name)

# A mode WE requested must not be mistaken for pilot takeover.
fresh(DroneState.RTL, enable_us=1900)
tick(1)
check("self-commanded RTL stays in RTL, not corrupted to IDLE",
      main.state == DroneState.RTL, "-> %s" % main.state.name)
del sent[:]
tick(5)
check("no setpoints sent while our own RTL executes", len(sent) == 0,
      "-> %d setpoints" % len(sent))


print("\n=== State machine: faults ===")

fresh(DroneState.TRACKING, enable_us=1900, persons=[CENTRED])
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
fake_drone._link_age = cfg.LINK_TIMEOUT_S + 1.0
del sent[:]
tick(2, persons=[CENTRED])
check("link loss -> EMERGENCY", main.state == DroneState.EMERGENCY,
      "-> %s" % main.state.name)
check("no setpoints sent into a dead link", len(sent) == 0,
      "-> %d setpoints" % len(sent))

# EMERGENCY ladder: hold, then RTL.
fresh(DroneState.EMERGENCY, enable_us=1900)
tick(2)
check("EMERGENCY holds zero velocity during the settle window",
      all(s == (0.0, 0.0, 0.0, 0.0) for s in sent), "-> %r" % sent[:3])
check("EMERGENCY does not escalate before EMERGENCY_HOLD_S",
      "RTL" not in mode_requests, "-> %r" % mode_requests)
CLOCK[0] += cfg.EMERGENCY_HOLD_S + 0.1
tick(1)
check("EMERGENCY escalates to RTL after the hold",
      "RTL" in mode_requests, "-> %r" % mode_requests)

# RTL rejected -> LAND fallback.
fresh(DroneState.EMERGENCY, enable_us=1900)
fake_drone.set_mode_ok = False
tick(1)                                    # sets emergency_since
CLOCK[0] += cfg.EMERGENCY_HOLD_S + 0.1     # then let the hold expire
tick(cfg.MAX_RTL_ATTEMPTS + 3)
check("EMERGENCY falls back to LAND when RTL is rejected",
      "LAND" in mode_requests, "-> %r" % mode_requests)

# RTL rejected from the RTL state -> EMERGENCY.
fresh(DroneState.RTL, enable_us=1900)
fake_drone.set_mode_ok = False
tick(1)
check("rejected RTL escalates to EMERGENCY",
      main.state == DroneState.EMERGENCY, "-> %s" % main.state.name)

# Vision stall must stop forward motion and eventually escalate.
fresh(DroneState.TRACKING, enable_us=1900, persons=[CENTRED])
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
del sent[:]
# Freeze the vision timestamp so it ages out.
for _ in range(5):
    CLOCK[0] += cfg.TICK_PERIOD_S
    main.run_tick(CLOCK[0], 0.001)     # fake_camera.ts left stale
check("stale vision commands zero forward velocity",
      all(s[0] == 0.0 for s in sent), "-> %r" % sent[:4])

fresh(DroneState.TRACKING, enable_us=1900, persons=None)
fake_camera.persons = None             # inference unavailable
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0]
for _ in range(cfg.MAX_VISION_FAULTS + 2):
    CLOCK[0] += cfg.TICK_PERIOD_S
    fake_camera.ts = CLOCK[0]
    main.run_tick(CLOCK[0], 0.001)
check("sustained vision fault -> EMERGENCY",
      main.state == DroneState.EMERGENCY, "-> %s" % main.state.name)
fake_camera.persons = []

# MAX_TRACK_DURATION cap.
fresh(DroneState.TRACKING, enable_us=1900, persons=[CENTRED])
main.last_confident_track = CLOCK[0]
main.track_started = CLOCK[0] - cfg.MAX_TRACK_DURATION_S - 1.0
tick(1, persons=[CENTRED])
check("MAX_TRACK_DURATION cap -> RTL", main.state == DroneState.RTL,
      "-> %s" % main.state.name)


print("\n=== Engagement gates have no bypass ===")

# Ground testing lives in bench.py, a separate program. There must be no flag,
# constant or environment variable in the flight code that can relax these.
check("no BENCH_MODE attribute exists in config",
      not hasattr(cfg, "BENCH_MODE"))
check("no bypass-looking environment variable is consulted",
      not any(k.startswith("BILBO_BENCH") for k in os.environ))

fresh(DroneState.READY, enable_us=1000, persons=[CENTRED])
fake_drone.armed = False
fake_drone.alt = 0.0
tick(2)
fake_drone.rc[cfg.CH_ENABLE] = 1900
tick(cfg.TARGET_CONFIRM_FRAMES + 4)
check("disarmed and grounded cannot engage under any configuration",
      main.state == DroneState.READY, "-> %s" % main.state.name)


print("\n=== drone.move(): final safety clamps ===")

import importlib
sys.modules.pop("drone")
real_drone = importlib.import_module("drone")


class FakeMav:
    def __init__(self):
        self.calls = []

    def set_position_target_local_ned_send(self, *args):
        self.calls.append(args)


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self):
        self.mav = FakeMav()


fm = FakeMaster()
real_drone.master = fm

real_drone._last_setpoint_tx = 0.0
real_drone.move(float("nan"), yaw_rate=float("inf"), force=True)
vx, vy, vz, yr = fm.mav.calls[-1][8], fm.mav.calls[-1][9], fm.mav.calls[-1][10], fm.mav.calls[-1][15]
check("NaN forward velocity is replaced with 0", vx == 0.0, "-> %r" % vx)
check("inf yaw rate is replaced with 0", yr == 0.0, "-> %r" % yr)

real_drone.move(999.0, yaw_rate=9999.0, force=True)
vx, yr = fm.mav.calls[-1][8], fm.mav.calls[-1][15]
import math as _m
check("absurd forward velocity clamped to HARD_MAX_SPEED",
      vx == cfg.HARD_MAX_SPEED_MS, "-> %r" % vx)
check("absurd yaw rate clamped to HARD_MAX_YAW_RATE",
      abs(_m.degrees(yr) - cfg.HARD_MAX_YAW_RATE_DEG_S) < 1e-6,
      "-> %.2f deg/s" % _m.degrees(yr))

real_drone.move(None, yaw_rate=None, force=True)
check("None velocities are replaced with 0",
      fm.mav.calls[-1][8] == 0.0 and fm.mav.calls[-1][15] == 0.0)

# Rate limiting must drop excess sends but never the forced safe-stop.
before = len(fm.mav.calls)
for _ in range(20):
    real_drone.move(0.5, force=False)
check("setpoint rate limiting drops excess sends",
      len(fm.mav.calls) - before <= 2,
      "-> %d sends for 20 calls" % (len(fm.mav.calls) - before))

before = len(fm.mav.calls)
for _ in range(5):
    real_drone.stop()
check("forced safe-stop is never rate limited",
      len(fm.mav.calls) - before == 5,
      "-> %d sends for 5 stops" % (len(fm.mav.calls) - before))

# The frame and mask must be exactly what ArduPilot expects.
check("velocity frame is BODY_NED",
      fm.mav.calls[-1][3] == 8, "-> %r" % fm.mav.calls[-1][3])
check("bitmask is the velocity+yaw_rate mask (1479)",
      fm.mav.calls[-1][4] == 0b010111000111 == 1479,
      "-> %d" % fm.mav.calls[-1][4])

raise SystemExit(summary())
