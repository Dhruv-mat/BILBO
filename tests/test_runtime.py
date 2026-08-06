"""Actually execute every entry point, rather than only unit-testing pieces.

The `cfg.LIDAR_VGATE_PX` crash in bench.py survived a full unit suite and a
clean pyflakes run because nothing had ever *run* cmd_track's inner loop. Static
checks cannot see a stale attribute reference inside a branch, so this file
drives the real loops -- bench subcommands, main()'s tick loop, preflight
failure paths, the CSV writer, and every LED state -- with the hardware stubbed.
"""

import sys
import time
import types

import harness  # noqa: F401  installs hardware stubs
from harness import check, summary, FakePerson, tf_frame

import config as cfg

# ---------------------------------------------------------------- fakes -----

sent = []
mode_requests = []


class FakeMav:
    def __init__(self):
        self.calls = []

    def set_position_target_local_ned_send(self, *a):
        self.calls.append(("setpoint", a))

    def heartbeat_send(self, *a):
        self.calls.append(("heartbeat", a))

    def command_long_send(self, *a):
        self.calls.append(("command", a))

    def request_data_stream_send(self, *a):
        self.calls.append(("stream", a))


class FakeMsg:
    def __init__(self, name, **fields):
        self._name = name
        self.__dict__.update(fields)

    def get_type(self):
        return self._name


class FakeMaster:
    """Enough of pymavlink's mavfile to drive drone.py for real."""

    target_system = 1
    target_component = 1

    def __init__(self, queue=None, mode="GUIDED"):
        self.mav = FakeMav()
        self.flightmode = mode
        self.queue = list(queue or [])
        self.closed = False

    def recv_match(self, blocking=False, timeout=None, type=None, **kw):
        while self.queue:
            msg = self.queue.pop(0)
            if type is None:
                return msg
            wanted = [type] if isinstance(type, str) else type
            if msg.get_type() in wanted:
                return msg
        return None

    def mode_mapping(self):
        return {"GUIDED": 4, "RTL": 6, "LAND": 9, "STABILIZE": 0}

    def set_mode(self, mode_id):
        mode_requests.append(mode_id)

    def close(self):
        self.closed = True


def heartbeat(armed=True):
    return FakeMsg("HEARTBEAT",
                   base_mode=(128 if armed else 0),
                   custom_mode=4)


def rc(ch6=1000):
    fields = {"chan%d_raw" % i: 1500 for i in range(1, 9)}
    fields["chan6_raw"] = ch6
    return FakeMsg("RC_CHANNELS", **fields)


# =========================================================== drone.py ======

print("\n=== drone.py: the real receive/send paths ===")

import drone  # noqa: E402

drone.master = FakeMaster(queue=[
    heartbeat(armed=True),
    rc(ch6=1750),
    FakeMsg("GLOBAL_POSITION_INT", relative_alt=4500),
    FakeMsg("BAD_DATA"),
    FakeMsg("ATTITUDE", roll=0.0),
])
n = drone.poll()
check("poll() drains the whole queue", n == 5, "-> %d messages" % n)
check("HEARTBEAT populated mode and armed",
      drone.get_mode() == "GUIDED" and drone.is_armed() is True,
      "-> %s armed=%s" % (drone.get_mode(), drone.is_armed()))
check("RC_CHANNELS populated ch6", drone.get_channel(6) == 1750,
      "-> %r" % drone.get_channel(6))
check("GLOBAL_POSITION_INT converted mm to metres",
      abs(drone.get_relative_alt() - 4.5) < 1e-9,
      "-> %r" % drone.get_relative_alt())
check("BAD_DATA did not raise", True)
check("link_age is fresh after a heartbeat", drone.link_age() < 1.0,
      "-> %.3f s" % drone.link_age())

# A flooded link must not make poll() loop forever.
drone.master = FakeMaster(queue=[heartbeat() for _ in range(10000)])
n = drone.poll(max_msgs=200)
check("poll() is bounded so a flooded link cannot hang the loop", n == 200,
      "-> %d" % n)

# Disarmed heartbeat must clear the armed flag.
drone.master = FakeMaster(queue=[heartbeat(armed=False)])
drone.poll()
check("disarmed heartbeat clears the armed flag", drone.is_armed() is False)

# set_mode: succeeds only when the vehicle confirms.
del mode_requests[:]
master = FakeMaster(mode="GUIDED")
drone.master = master


class ConfirmingMaster(FakeMaster):
    """Reports the new mode once set_mode has been issued, like a real vehicle."""

    def __init__(self):
        super().__init__(mode="GUIDED")
        self.requested = None

    def set_mode(self, mode_id):
        mode_requests.append(mode_id)
        self.requested = mode_id

    def recv_match(self, blocking=False, timeout=None, type=None, **kw):
        if self.requested == 6:
            self.flightmode = "RTL"
        return heartbeat()


drone.master = ConfirmingMaster()
ok = drone.set_mode("RTL", timeout=1.0, retries=0)
check("set_mode returns True when the vehicle confirms", ok is True)
check("set_mode updated the cached mode", drone.get_mode() == "RTL",
      "-> %s" % drone.get_mode())


class SilentMaster(FakeMaster):
    """Never changes mode -- the deadlock case the original hung on forever."""

    def recv_match(self, blocking=False, timeout=None, type=None, **kw):
        return heartbeat()


drone.master = SilentMaster(mode="GUIDED")
t0 = time.monotonic()
ok = drone.set_mode("RTL", timeout=0.3, retries=1)
elapsed = time.monotonic() - t0
check("set_mode returns False instead of hanging when mode never changes",
      ok is False, "-> %r" % ok)
check("set_mode respects its timeout budget", elapsed < 2.0,
      "-> %.2f s" % elapsed)

check("set_mode rejects an unknown mode",
      drone.set_mode("NOT_A_MODE") is False)

# Unknown modes and disconnected state must not raise.
drone.master = None
check("set_mode while disconnected returns False",
      drone.set_mode("RTL") is False)
check("poll while disconnected returns 0", drone.poll() == 0)
check("move while disconnected returns False", drone.move(1.0) is False)
check("link_age is infinite while disconnected",
      drone.link_age() == float("inf"))
check("send_heartbeat while disconnected returns False",
      drone.send_heartbeat() is False)
try:
    drone.get_channel(6)
    drone.get_rc_channels()
    drone.get_relative_alt()
    _ok, _err = True, ""
except Exception as _exc:
    _ok, _err = False, " -> %r" % _exc
check("cache reads while disconnected do not raise", _ok, _err)

# request_streams must issue one command per wanted message plus the fallback.
drone.master = FakeMaster()
drone._last_heartbeat_tx = 0.0
drone.request_streams()
cmds = [c for c in drone.master.mav.calls if c[0] == "command"]
streams = [c for c in drone.master.mav.calls if c[0] == "stream"]
check("request_streams asks for each required message", len(cmds) == 4,
      "-> %d commands" % len(cmds))
check("request_streams also sends the legacy data-stream fallback",
      len(streams) == 1)

# Heartbeat is rate limited.
drone._last_heartbeat_tx = 0.0
first = drone.send_heartbeat()
second = drone.send_heartbeat()
check("send_heartbeat emits once then rate limits", first and not second,
      "-> %r then %r" % (first, second))

# =========================================================== main.py ======

print("\n=== main.py: CSV writer and preflight failure paths ===")

fake_camera = types.ModuleType("camera")
fake_camera.get_latest = lambda: (time.monotonic(), [])
fake_camera.frame_age = lambda: 0.0
fake_camera.inference_alive = lambda: True
fake_camera.health = lambda: {}
fake_camera.stop = lambda: None
fake_camera.intitalise = lambda: (cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)
sys.modules["camera"] = fake_camera

import main  # noqa: E402
from state import DroneState  # noqa: E402

# The CSV writer must survive every record shape, including the sparse one
# written when a tick raised.
rows = []


class FakeWriter:
    def writerow(self, row):
        rows.append(row)


main._csv_writer = FakeWriter()
main._csv_file = types.SimpleNamespace(flush=lambda: None)
drone.master = FakeMaster(queue=[heartbeat()])
drone.poll()

full = {
    "state": "TRACKING", "enable": "FULL", "mode": "GUIDED", "armed": True,
    "alt": 5.0, "vision_age": 0.02, "n_persons": 2, "error_x": 12.5,
    "error_y": -3.0, "bearing_deg": 1.5, "distance_cm": 410,
    "yaw_rate": 2.5, "forward": 0.4, "gate": "open", "tick_s": 0.01,
}
sparse = {"state": "TRACKING", "gate": "exception", "tick_s": 0.02}
empty = {}

for label, record in (("full", full), ("sparse/exception", sparse),
                      ("completely empty", empty)):
    before = len(rows)
    try:
        main._write_record(record, time.monotonic())
        ok = len(rows) == before + 1
        err = ""
    except Exception as exc:
        ok, err = False, " -> raised %r" % exc
    check("CSV writer handles a %s record" % label, ok, err)

check("all CSV rows have the same column count as the header",
      all(len(r) == len(main.CSV_COLUMNS) for r in rows),
      "-> widths %r vs %d columns"
      % (sorted(set(len(r) for r in rows)), len(main.CSV_COLUMNS)))

main._csv_writer = None
main._csv_file = None

# Preflight must fail cleanly, not raise, when the link is absent.
drone_connect = drone.connect
drone.connect = lambda timeout=10.0: False
check("preflight returns False (not an exception) when the FC is absent",
      main.preflight() is False)
drone.connect = drone_connect

# ...and when the LiDAR cannot be opened.
import lidar  # noqa: E402
lidar_init = lidar.init


def boom():
    raise OSError("device or resource busy")


drone.connect = lambda timeout=10.0: True
drone.master = FakeMaster(queue=[heartbeat()])
lidar.init = boom
check("preflight returns False when the LiDAR port is busy",
      main.preflight() is False)
lidar.init = lidar_init
drone.connect = drone_connect

# soft_reset must clear everything, and keep_target must spare the tracker.
import tracker  # noqa: E402

tracker.configure(cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)
tracker.reset()
tracker.select([FakePerson(320, 240, 40000)])
main.last_dist, main.last_dist_t = 400, time.monotonic()
main.lost_since = 1.0
main.soft_reset(keep_target=True)
check("soft_reset(keep_target=True) spares the tracker identity",
      tracker.has_target())
check("soft_reset cleared the retained range", main.last_dist is None)
check("soft_reset cleared the loss timer", main.lost_since is None)
main.soft_reset()
check("soft_reset() without keep_target clears the tracker",
      not tracker.has_target())

# =========================================================== led.py ======

print("\n=== led.py: every state renders without raising ===")

import led  # noqa: E402

led.init()
for st in DroneState:
    try:
        led.render(st)
        led.render(st, fault="demo")
        ok, err = True, ""
    except Exception as exc:
        ok, err = False, " -> %r" % exc
    check("render(%s) does not raise" % st.name, ok, err)

check("every DroneState has an appearance mapping",
      all(st in led._STATE_APPEARANCE for st in DroneState),
      "-> missing %r" % [st.name for st in DroneState
                         if st not in led._STATE_APPEARANCE])
check("every mapped colour exists in the colour table",
      all(c in led.colours for _, c in led._STATE_APPEARANCE.values()))

# LED failure must never propagate into the flight loop.
led._available = True


class ExplodingNeo:
    def fill_strip(self, r, g, b):
        raise OSError("SPI gone")

    def update_strip(self):
        pass


led.neo = ExplodingNeo()
led._last_written = None
try:
    led.render(DroneState.TRACKING)
    ok, err = True, ""
except Exception as exc:
    ok, err = False, " -> %r" % exc
check("an SPI failure mid-render is swallowed, not raised", ok, err)
check("and LED output disables itself after failing", not led.is_available())

# =========================================================== bench.py ======

print("\n=== bench.py: every subcommand's real loop ===")

fake_camera.picam2 = types.SimpleNamespace(pre_callback=None)
fake_camera._on_frame = lambda request: None

import bench  # noqa: E402

from pymavlink import mavutil  # noqa: E402
if not hasattr(mavutil.mavlink, "MAV_CMD_DO_MOTOR_TEST"):
    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST = 209
if not hasattr(mavutil.mavlink, "MAV_RESULT_ACCEPTED"):
    mavutil.mavlink.MAV_RESULT_ACCEPTED = 0

# Break out of the infinite loops after a fixed number of iterations by making
# sleep raise, which is exactly how Ctrl-C would land.
real_sleep = time.sleep


class Ticker:
    def __init__(self, limit):
        self.n = 0
        self.limit = limit

    def __call__(self, seconds):
        self.n += 1
        if self.n >= self.limit:
            raise KeyboardInterrupt


def run_loop(fn, args, iterations=25):
    ticker = Ticker(iterations)
    time.sleep = ticker
    try:
        rc_ = fn(args)
        return rc_, ticker.n, None
    except BaseException as exc:      # must NOT happen
        return None, ticker.n, exc
    finally:
        time.sleep = real_sleep


drone.connect = lambda timeout=10.0: True
drone.master = FakeMaster(queue=[heartbeat(), rc(1750)])
drone.poll()

# cmd_track: the loop that had the latent crash. Feed real LiDAR frames and a
# person that drifts, so the range gate opens and closes.
lidar.init()
for _ in range(6):
    lidar.ser.feed(tf_frame(420))

drifting = [FakePerson(320 + i * 4, 240, 40000) for i in range(40)]
seq = {"i": 0}


def moving_latest():
    i = seq["i"]
    seq["i"] += 1
    if i % 7 == 6:
        return (time.monotonic(), [])            # a frame with nobody
    if i % 11 == 10:
        return (time.monotonic(), None)          # inference unavailable
    lidar.ser.feed(tf_frame(400 + (i % 5) * 3))
    return (time.monotonic(), [drifting[i % len(drifting)]])


fake_camera.get_latest = moving_latest
tracker.reset()

args = types.SimpleNamespace(send=False, yaw_only=False, no_preview=True,
                             conf=None, min_area=None, hgate_frac=None)
rc_, iters, exc = run_loop(bench.cmd_track, args, iterations=30)
check("bench track runs its full loop without raising",
      exc is None, "-> %r" % exc)
check("bench track completed many iterations", iters >= 25, "-> %d" % iters)

# Same loop with the tuning overrides applied.
seq["i"] = 0
tracker.reset()
args = types.SimpleNamespace(send=False, yaw_only=True, no_preview=True,
                             conf=0.30, min_area=400, hgate_frac=0.45)
saved = (cfg.CONF_THRESHOLD, cfg.MIN_BOX_AREA_PX, cfg.LIDAR_HGATE_FRAC)
rc_, iters, exc = run_loop(bench.cmd_track, args, iterations=15)
check("bench track --conf/--min-area/--hgate-frac overrides run cleanly",
      exc is None, "-> %r" % exc)
check("overrides actually took effect",
      cfg.CONF_THRESHOLD == 0.30 and cfg.MIN_BOX_AREA_PX == 400
      and cfg.LIDAR_HGATE_FRAC == 0.45)
cfg.CONF_THRESHOLD, cfg.MIN_BOX_AREA_PX, cfg.LIDAR_HGATE_FRAC = saved

# cmd_switch
drone.master = FakeMaster(queue=[heartbeat(), rc(1500)])
rc_, iters, exc = run_loop(bench.cmd_switch,
                           types.SimpleNamespace(), iterations=10)
check("bench switch runs without raising", exc is None, "-> %r" % exc)

# cmd_sensors
lidar.close()
for _ in range(3):
    pass
rc_, iters, exc = run_loop(bench.cmd_sensors,
                           types.SimpleNamespace(), iterations=10)
check("bench sensors runs without raising", exc is None, "-> %r" % exc)

# cmd_leds paces itself off monotonic time, not sleep counts, and its
# liveness demo deliberately spans two blip periods. Shorten the period so
# the walk completes quickly instead of busy-spinning for seconds.
led._available = True
led.neo = harness.FakeNeo("/dev/spidev0.0")
saved_period = led.LIVENESS_PERIOD_S
led.LIVENESS_PERIOD_S = 0.02
time.sleep = lambda s: None
try:
    code = bench.cmd_leds(types.SimpleNamespace(dwell=0.01))
    ok, err = code == 0, "-> returned %r" % code
except BaseException as exc:
    ok, err = False, " -> %r" % exc
finally:
    time.sleep = real_sleep
    led.LIVENESS_PERIOD_S = saved_period
check("bench leds walks every state and returns 0", ok, err)

# Ctrl-C during the LED walk must blank the strip, not leave it lit.
led._available = True
led.neo = harness.FakeNeo("/dev/spidev0.0")
led._last_written = None
walk = bench._walk_led_states


def interrupted(*a, **kw):
    raise KeyboardInterrupt


bench._walk_led_states = interrupted
try:
    code = bench.cmd_leds(types.SimpleNamespace(dwell=0.01))
    ok = code == 130 and led._last_written == led.colours["off"]
    err = "-> code %r, last write %r" % (code, led._last_written)
except BaseException as exc:
    ok, err = False, " -> %r" % exc
finally:
    bench._walk_led_states = walk
check("Ctrl-C during the LED walk blanks the strip", ok, err)

# cmd_link
drone.master = FakeMaster(queue=[heartbeat(), rc(1750),
                                 FakeMsg("GLOBAL_POSITION_INT",
                                         relative_alt=1000)])


class LinkMaster(FakeMaster):
    """Streams forever so cmd_link's measuring window has traffic."""

    def recv_match(self, blocking=False, timeout=None, type=None, **kw):
        self.queue.extend([heartbeat(), rc(1750)])
        return self.queue.pop(0)


drone.master = LinkMaster()
try:
    code = bench.cmd_link(types.SimpleNamespace(seconds=0.2))
    ok, err = True, ""
except Exception as exc:
    ok, err = False, " -> %r" % exc
check("bench link runs and returns a code", ok, err)

# cmd_motors: the confirmation gate, driven for real.
import builtins  # noqa: E402

real_input = builtins.input


def scripted(*answers):
    it = iter(answers)
    builtins.input = lambda prompt="": next(it, "")


drone.master = FakeMaster()
drone.is_armed = lambda: False
motor_args = types.SimpleNamespace(throttle=15, duration=0.01, count=4,
                                   motor=None, yaw_right=False, yaw_left=False)
scripted("REMOVED", "", "", "", "")
time.sleep = lambda s: None
mav = drone.master.mav      # cmd_motors disconnects when it finishes
try:
    code = bench.cmd_motors(motor_args)
    cmds = [c for c in mav.calls if c[0] == "command"]
    ok, err = len(cmds) == 4, "-> %d motor commands" % len(cmds)
except Exception as exc:
    ok, err = False, " -> %r" % exc
finally:
    time.sleep = real_sleep
    builtins.input = real_input
check("bench motors spins each motor once after confirmation", ok, err)

drone.master = FakeMaster()
motor_args.yaw_right = True
scripted("REMOVED", "", "")
time.sleep = lambda s: None
mav = drone.master.mav
try:
    bench.cmd_motors(motor_args)
    motors = [c[1][4] for c in mav.calls if c[0] == "command"]
    ok = motors == [float(m) for m in bench.YAW_RIGHT_MOTORS]
    err = "-> %r" % motors
except Exception as exc:
    ok, err = False, " -> %r" % exc
finally:
    time.sleep = real_sleep
    builtins.input = real_input
check("bench motors --yaw-right spins exactly the nose-right pair", ok, err)

check("yaw_hint agrees with the motor pairs",
      "M%d,M%d" % bench.YAW_RIGHT_MOTORS in bench.yaw_hint(10.0)
      and "M%d,M%d" % bench.YAW_LEFT_MOTORS in bench.yaw_hint(-10.0),
      "-> %r / %r" % (bench.yaw_hint(10.0), bench.yaw_hint(-10.0)))

lidar.close()

# ===================================================== main() loop ========

print("\n=== main(): exception containment and the stall watchdog ===")


def drive_main(tick_impl, ticks=8, preflight_ok=True):
    """Run main()'s real loop for a bounded number of ticks."""
    saved = (main.run_tick, main.preflight, main._shutdown, main.state,
             main.consecutive_faults, main.fault_latch, drone.stop,
             main._open_csv, led.note_tick, led.render)
    calls = {"n": 0, "stops": 0}

    def counting_stop(force=True):
        calls["stops"] += 1
        return True

    def wrapped(now, tick_duration):
        calls["n"] += 1
        if calls["n"] >= ticks:
            main._shutdown = True
        return tick_impl(now, tick_duration)

    main.run_tick = wrapped
    main.preflight = lambda: preflight_ok
    main._open_csv = lambda: None
    main._shutdown = False
    main.state = DroneState.IDLE
    main.consecutive_faults = 0
    main.fault_latch = None
    drone.stop = counting_stop
    led.note_tick = lambda: None
    led.render = lambda st, fault=None: None
    try:
        code = main.main()
        return code, calls, main.state, main.fault_latch, None
    except BaseException as exc:
        return None, calls, main.state, main.fault_latch, exc
    finally:
        (main.run_tick, main.preflight, main._shutdown, main.state,
         main.consecutive_faults, main.fault_latch, drone.stop,
         main._open_csv, led.note_tick, led.render) = saved


# A healthy loop.
code, calls, st, latch, exc = drive_main(
    lambda now, td: {"state": "IDLE", "gate": "-", "tick_s": td}, ticks=6)
check("main() runs its loop and exits cleanly", exc is None and code == 0,
      "-> code %r exc %r" % (code, exc))
check("main() ran the expected number of ticks", calls["n"] == 6,
      "-> %d" % calls["n"])
check("main() commanded zero velocity on shutdown", calls["stops"] >= 1,
      "-> %d stop calls" % calls["stops"])


# A tick that always raises must not kill the loop, and must escalate.
def always_raises(now, td):
    raise ValueError("synthetic tick fault")


code, calls, st, latch, exc = drive_main(always_raises, ticks=10)
check("a tick that raises every time does not terminate the loop",
      exc is None and calls["n"] == 10, "-> %d ticks, exc %r"
      % (calls["n"], exc))
check("each faulting tick commands a safe stop first",
      calls["stops"] >= 10, "-> %d stop calls for 10 faults" % calls["stops"])
check("repeated faults escalate to EMERGENCY",
      st == DroneState.EMERGENCY, "-> %s" % st.name)
check("repeated faults latch a fault flag", latch == "faults",
      "-> %r" % latch)


# One fault then recovery must clear the latch rather than sticking.
seq2 = {"n": 0}


def fails_once(now, td):
    seq2["n"] += 1
    if seq2["n"] <= cfg.MAX_CONSECUTIVE_FAULTS:
        raise ValueError("transient")
    return {"state": "IDLE", "gate": "-", "tick_s": td}


code, calls, st, latch, exc = drive_main(fails_once, ticks=12)
check("a fault latch clears once ticks succeed again", latch is None,
      "-> %r" % latch)


# A tick that overruns the watchdog must escalate to EMERGENCY.
def slow_tick(now, td):
    time.sleep(cfg.TICK_OVERRUN_LIMIT_S + 0.05)
    return {"state": "IDLE", "gate": "-", "tick_s": td}


code, calls, st, latch, exc = drive_main(slow_tick, ticks=2)
check("a tick overrunning the watchdog escalates to EMERGENCY",
      st == DroneState.EMERGENCY, "-> %s" % st.name)

# Preflight failure must return non-zero and never enter the loop.
code, calls, st, latch, exc = drive_main(
    lambda now, td: {}, ticks=5, preflight_ok=False)
check("preflight failure returns non-zero", code == 1, "-> %r" % code)
check("preflight failure never enters the tick loop", calls["n"] == 0,
      "-> %d ticks" % calls["n"])

raise SystemExit(summary())
