"""Exercise the real BILBO logic modules with hardware stubbed out."""

import harness
from harness import check, summary, tf_frame, FakePerson

import config as cfg
import lidar
import tracker
import controller
import led

print("\n=== LiDAR: framing, checksum, newest-frame, resync ===")

lidar.init()
ser = lidar.ser

# Mid-frame start: the original desynced permanently on exactly this.
ser.feed(b"\x12\x34\x56" + tf_frame(150) + tf_frame(220) + tf_frame(310))
d = lidar.read_data()
check("newest frame wins after garbage prefix", d == 310, "-> %r" % d)

ser.feed(tf_frame(400, corrupt=True))
before = lidar.health()["bad_checksum"]
d = lidar.read_data()
check("corrupt checksum rejected", d is None, "-> %r" % d)
check("bad_checksum counter incremented",
      lidar.health()["bad_checksum"] > before)

ser.feed(tf_frame(500, strength=10))
d = lidar.read_data()
check("low signal strength rejected", d is None, "-> %r" % d)

ser.feed(tf_frame(500, strength=65535))
d = lidar.read_data()
check("saturated strength rejected", d is None, "-> %r" % d)

ser.feed(tf_frame(5))       # below LIDAR_MIN_CM
d = lidar.read_data()
check("out-of-range low rejected", d is None, "-> %r" % d)

ser.feed(tf_frame(900))      # above LIDAR_MAX_CM
d = lidar.read_data()
check("out-of-range high rejected", d is None, "-> %r" % d)

# Resync after arbitrary mid-stream corruption.
good = tf_frame(275)
ser.feed(good[:4] + b"\xAA\xBB" + good + b"\x00")
d = lidar.read_data()
check("resyncs mid-stream", d == 275, "-> %r" % d)

# Buffer must stay bounded even if nothing valid ever arrives.
for _ in range(200):
    ser.feed(b"\xAA" * 32)
    lidar.read_data()
check("rx buffer bounded", len(lidar._buf) <= cfg.LIDAR_RX_BUFFER_CAP,
      "-> %d bytes" % len(lidar._buf))

# Partial trailing frame must be retained, not discarded.
ser.feed(tf_frame(180)[:5])
lidar.read_data()
ser.feed(tf_frame(180)[5:])
d = lidar.read_data()
check("split frame reassembled across calls", d == 180, "-> %r" % d)


print("\n=== Controller: yaw sign chain (the confirmed runaway) ===")

controller.reset()
t = controller.update(+100.0, None, send=False)
check("target RIGHT (error_x>0) -> yaw POSITIVE (nose right)",
      t["yaw_rate"] > 0, "yaw=%+.2f" % t["yaw_rate"])

controller.reset()
t = controller.update(-100.0, None, send=False)
check("target LEFT (error_x<0) -> yaw NEGATIVE (nose left)",
      t["yaw_rate"] < 0, "yaw=%+.2f" % t["yaw_rate"])

controller.reset()
t = controller.update(0.0, None, send=False)
check("centred -> zero yaw", abs(t["yaw_rate"]) < 1e-9,
      "yaw=%+.2f" % t["yaw_rate"])

# Deadband must be continuous: just outside it the output is small, not a step.
controller.reset()
t_in = controller.update(cfg.YAW_DEADBAND_PX - 1, None, send=False)
controller.reset()
t_out = controller.update(cfg.YAW_DEADBAND_PX + 1, None, send=False)
check("no box width falls back to the fixed deadband",
      controller.deadband_px(None) == cfg.YAW_DEADBAND_PX)
check("deadband scales with box width",
      controller.deadband_px(300) > controller.deadband_px(80),
      "300px box -> %.0f, 80px box -> %.0f"
      % (controller.deadband_px(300), controller.deadband_px(80)))
check("deadband edge is continuous (no 5 deg/s step)",
      abs(t_in["yaw_rate"]) < 1e-9 and abs(t_out["yaw_rate"]) < 0.5,
      "in=%.3f out=%.3f" % (t_in["yaw_rate"], t_out["yaw_rate"]))

# Yaw rate must respect the limit even at huge error.
controller.reset()
t = controller.update(10000.0, None, send=False)
check("yaw rate clamped to MAX_YAW_RATE",
      abs(t["yaw_rate"]) <= cfg.MAX_YAW_RATE_DEG_S + 1e-9,
      "yaw=%+.2f limit=%.1f" % (t["yaw_rate"], cfg.MAX_YAW_RATE_DEG_S))


print("\n=== Controller: forward gating ===")

controller.reset()
t = controller.update(0.0, None, send=False)
check("no distance -> forward 0, gate=no_distance",
      t["forward"] == 0.0 and t["gate"] == "no_distance",
      "fwd=%.2f gate=%s" % (t["forward"], t["gate"]))

controller.reset()
t = controller.update(0.0, 600.0, send=False, yaw_only=True)
check("yaw_only -> forward 0, gate=yaw_only",
      t["forward"] == 0.0 and t["gate"] == "yaw_only",
      "fwd=%.2f gate=%s" % (t["forward"], t["gate"]))

# 300 px / 8.2 px-per-deg = 36.6 deg, well beyond the 15 deg limit.
controller.reset()
t = controller.update(300.0, 600.0, send=False)
check("misaligned -> forward 0, gate=misaligned",
      t["forward"] == 0.0 and t["gate"] == "misaligned",
      "bearing=%.1f deg fwd=%.2f gate=%s"
      % (t["bearing_deg"], t["forward"], t["gate"]))

def drive(error_x, distance_cm, ticks=60, yaw_only=False, t0=1000.0):
    """Run the controller over simulated ticks at the real loop period."""
    controller.reset()
    telemetry = None
    for i in range(ticks):
        telemetry = controller.update(
            error_x, distance_cm, yaw_only=yaw_only, send=False,
            now=t0 + i * cfg.TICK_PERIOD_S,
        )
    return telemetry

# Aligned and far: forward must ramp POSITIVE (toward the target).
t = drive(0.0, 600.0)
check("aligned + far -> forward positive (approach)", t["forward"] > 0.5,
      "fwd=%.3f gate=%s" % (t["forward"], t["gate"]))
check("forward clamped to MAX_FORWARD_SPEED",
      t["forward"] <= cfg.MAX_FORWARD_SPEED_MS + 1e-9, "fwd=%.3f" % t["forward"])

# Aligned and too close: must retreat.
t = drive(0.0, 200.0)
check("aligned + closer than target -> forward negative (retreat)",
      t["forward"] < -0.01, "fwd=%.3f" % t["forward"])

# Inside the hard floor: retreat regardless of what the tracker wants.
t = drive(0.0, 100.0)
check("inside MIN_SAFE_DISTANCE -> backing off",
      t["backing_off"] and t["forward"] <= -cfg.BACKOFF_SPEED_MS + 1e-9,
      "fwd=%.3f backing_off=%s" % (t["forward"], t["backing_off"]))

# Even misaligned, proximity must still trigger the back-off.
t = drive(400.0, 100.0)
check("proximity back-off applies even when misaligned",
      t["forward"] <= -cfg.BACKOFF_SPEED_MS + 1e-9, "fwd=%.3f" % t["forward"])

# yaw_only must hold forward at zero for the whole run, not just tick 1.
t = drive(0.0, 600.0, yaw_only=True)
check("yaw_only holds forward at zero across many ticks",
      t["forward"] == 0.0, "fwd=%.3f" % t["forward"])

# Slew: the first tick after a reset must not command a step.
controller.reset()
t = controller.update(0.0, 800.0, send=False, now=2000.0)
check("first tick after reset does not step forward velocity",
      abs(t["forward"]) < 1e-6, "fwd=%.4f" % t["forward"])

# Slew rate must be respected: one tick cannot exceed FORWARD_SLEW_MS2 * dt.
controller.reset()
controller.update(0.0, 800.0, send=False, now=3000.0)
t = controller.update(0.0, 800.0, send=False, now=3000.0 + cfg.TICK_PERIOD_S)
max_step = cfg.FORWARD_SLEW_MS2 * cfg.TICK_PERIOD_S
check("forward slew rate respected on a single tick",
      abs(t["forward"]) <= max_step + 1e-9,
      "fwd=%.4f max_step=%.4f" % (t["forward"], max_step))

# Reduction toward zero must be immediate, not slew-limited: a delayed stop is
# a safety problem.
controller.reset()
for i in range(60):
    controller.update(0.0, 600.0, send=False, now=4000.0 + i * cfg.TICK_PERIOD_S)
t = controller.update(0.0, None, send=False, now=4000.0 + 60 * cfg.TICK_PERIOD_S)
check("loss of range stops forward motion immediately",
      t["forward"] == 0.0, "fwd=%.4f" % t["forward"])


print("\n=== Tracker: persistence and lock gating ===")

tracker.configure(cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)
tracker.reset()

near = FakePerson(200, 240, 40000)
far = FakePerson(500, 240, 9000)
picked = tracker.select([far, near])
check("designates the largest box initially", picked is near, "-> %r" % picked)

# Same two people, both drifted slightly: must stay on the same one.
near2 = FakePerson(215, 245, 41000)
far2 = FakePerson(505, 240, 9200)
picked = tracker.select([far2, near2])
check("stays locked on the same person across frames",
      picked is near2, "-> %r" % picked)

# A stranger appears much closer but far away in image space: must NOT steal.
stranger = FakePerson(600, 240, 90000)
picked = tracker.select([stranger, FakePerson(220, 246, 41000)])
check("closer stranger outside the gate cannot steal the lock",
      picked is not None and abs(picked.center_x - 220) < 1.0,
      "-> %r" % picked)

# Target teleports beyond the gate: report loss instead of jumping.
picked = tracker.select([FakePerson(20, 470, 41000)])
check("out-of-gate jump reports loss rather than jumping",
      picked is None, "-> %r" % picked)

# Empty detections -> no target.
check("no detections -> None", tracker.select([]) is None)

# Lock gating: both axes.
tracker.reset()
centred = FakePerson(cfg.IMAGE_WIDTH_PX / 2, cfg.LIDAR_BORESIGHT_ROW_PX, 40000)
tracker.select([centred])
locked, ex, ey = tracker.is_locked(centred, cfg.IMAGE_WIDTH_PX / 2.0)
check("centred target is locked", locked and abs(ex) < 1 and abs(ey) < 1,
      "locked=%s ex=%.1f ey=%.1f" % (locked, ex, ey))

# Range gates scale with the target box: the beam lands on the person if it
# falls anywhere across their body, and the box measures that width.
wide = FakePerson(cfg.IMAGE_WIDTH_PX / 2, cfg.LIDAR_BORESIGHT_ROW_PX, 40000)
narrow = FakePerson(cfg.IMAGE_WIDTH_PX / 2, cfg.LIDAR_BORESIGHT_ROW_PX, 2500)
check("a wide (close) box gets a wider range gate than a narrow (far) one",
      tracker.hgate_px(wide.width) > tracker.hgate_px(narrow.width),
      "wide=%.0f px (w=%d)  narrow=%.0f px (w=%d)"
      % (tracker.hgate_px(wide.width), wide.width,
         tracker.hgate_px(narrow.width), narrow.width))
check("range gate is clamped at both ends",
      tracker.hgate_px(1) == cfg.LIDAR_HGATE_MIN_PX
      and tracker.hgate_px(100000) == cfg.LIDAR_HGATE_MAX_PX)

# THE INTERACTION THAT MUST NOT BREAK: if the yaw deadband were ever wider
# than the range gate, yaw would stop correcting while the target sat outside
# the beam, so no range would ever be read and forward velocity could never
# engage. The drone would park with the person off to one side forever.
worst = float("-inf")
for box_w in range(1, 641):
    db = controller.deadband_px(box_w)
    hg = tracker.hgate_px(box_w)
    worst = max(worst, db - hg)
check("yaw deadband is ALWAYS inside the range gate, for every box width",
      worst < 0, "worst (deadband - gate) = %+.1f px" % worst)

# Off-axis by more than its own gate -> not locked.
off_h = FakePerson(cfg.IMAGE_WIDTH_PX / 2 + 250,
                   cfg.LIDAR_BORESIGHT_ROW_PX, 40000)
locked, ex, ey = tracker.is_locked(off_h, cfg.IMAGE_WIDTH_PX / 2.0)
check("horizontally off-axis target is NOT locked", not locked,
      "ex=%.1f gate=%.0f" % (ex, tracker.hgate_px(off_h.width)))

off_v = FakePerson(cfg.IMAGE_WIDTH_PX / 2,
                   cfg.LIDAR_BORESIGHT_ROW_PX + 300, 2500)
locked, ex, ey = tracker.is_locked(off_v, cfg.IMAGE_WIDTH_PX / 2.0)
check("vertically off-axis target is NOT locked (beam over the head)",
      not locked,
      "ey=%.1f gate=%.0f" % (ey, tracker.vgate_px(off_v.height)))

# Parallax: the unit bug made this ~100x too small.
lc_far = tracker.get_lock_center(10000.0)
lc_near = tracker.get_lock_center(160.0)
offset_near = abs(lc_near - cfg.IMAGE_WIDTH_PX / 2.0)
check("parallax correction is a sane magnitude at 1.6 m",
      5.0 < offset_near < 40.0, "offset=%.1f px" % offset_near)
check("parallax shrinks with distance",
      abs(lc_far - cfg.IMAGE_WIDTH_PX / 2.0) < offset_near,
      "far=%.2f near=%.2f px" % (abs(lc_far - cfg.IMAGE_WIDTH_PX / 2.0),
                                 offset_near))
check("get_lock_center handles None distance",
      tracker.get_lock_center(None) == cfg.IMAGE_WIDTH_PX // 2)


print("\n=== Tracker: self-heal after repeated association failures ===")

# Reproduce the permanent-loss bug: lock on, then have the person reappear
# far outside the association gate every frame. Without self-heal the stale
# target position is compared against forever and the person is never
# re-acquired -- which is exactly what bench.py exhibited.
tracker.reset()
tracker.select([FakePerson(100, 240, 40000)])
check("locked on initially", tracker.has_target())

far_away = FakePerson(600, 240, 40000)     # way beyond TRACK_GATE_PX
results = [tracker.select([far_away]) for _ in range(cfg.TRACK_MAX_MISSES)]
check("association initially rejects the displaced person",
      results[0] is None)
check("re-designates within TRACK_MAX_MISSES instead of losing them "
      "forever",
      results[-1] is not None,
      "-> recovered on frame %s of %d"
      % (next((i + 1 for i, r in enumerate(results) if r is not None),
              None), cfg.TRACK_MAX_MISSES))
check("miss counter resets after recovery", tracker.misses() == 0,
      "-> %d" % tracker.misses())

# An empty frame run must also clear the target so the next person is
# designated fresh rather than compared against a stale position.
tracker.reset()
tracker.select([FakePerson(100, 240, 40000)])
for _ in range(cfg.TRACK_MAX_MISSES):
    tracker.select([])
check("a long run of empty frames clears the target",
      not tracker.has_target())
picked = tracker.select([far_away])
check("next detection is designated fresh", picked is far_away)

# A brief single-frame gap must NOT drop the lock.
tracker.reset()
anchor = FakePerson(300, 240, 40000)
tracker.select([anchor])
tracker.select([])
check("one dropped frame does not clear the target", tracker.has_target())
picked = tracker.select([FakePerson(310, 245, 41000)])
check("and the same person re-associates normally after it",
      picked is not None and tracker.misses() == 0)


print("\n=== LED: solid holds, writes only on change ===")

ok = led.init()
check("led.init() succeeds against a pi5neo with no brightness kwarg",
      ok and led.neo is not None)
check("no brightness kwarg was passed to the constructor",
      led._hw_brightness is False)
check("fill_strip convention detected as r,g,b",
      led._fill_takes_tuple is False)
neo = led.neo
neo.writes.clear()
led._last_written = None

for _ in range(10):
    led.led_status("solid", "green")
check("solid writes once, not once per call", len(neo.writes) == 1,
      "-> %d writes" % len(neo.writes))
check("solid wrote green, brightness-scaled",
      neo.writes[0] == led._scaled(led.colours["green"]),
      "-> %r (expected %r)" % (neo.writes[0],
                               led._scaled(led.colours["green"])))
check("software brightness scaling is applied",
      led._hw_brightness or neo.writes[0] != led.colours["green"],
      "-> %r at brightness %.2f" % (neo.writes[0], cfg.LED_BRIGHTNESS))

led.led_status("solid", "red")
check("colour change triggers exactly one more write",
      len(neo.writes) == 2
      and neo.writes[-1] == led._scaled(led.colours["red"]),
      "-> %r" % (neo.writes,))

led.led_status("solid", "nonexistent-colour")
check("unknown colour is ignored, not crashed", len(neo.writes) == 2)

# Blink must alternate over time without sleeping.
states = set()
import time as _t
t0 = _t.monotonic()
while _t.monotonic() - t0 < led.BLINK_PERIOD_S * 2.2:
    led._last_written = None
    led.led_status("blink", "yellow")
    states.add(neo.writes[-1])
check("blink alternates between colour and off",
      led._scaled(led.colours["yellow"]) in states
      and (0, 0, 0) in states, "-> %r" % (states,))

# Liveness must depend on the control loop, not wall clock.
led._last_tick_time = 0.0
check("liveness flash suppressed when the loop has never ticked",
      not led._liveness_overrides(_t.monotonic()))
led.note_tick()
stale = _t.monotonic() + led.LIVENESS_MAX_AGE_S + 1.0
check("liveness flash suppressed when the loop has stalled",
      not led._liveness_overrides(stale))


print("\n=== Camera: inference-less frames must not clobber detections ===")

import camera


class FakeRequest:
    def get_metadata(self):
        return {}


req = FakeRequest()
_real_parse = camera.parse_people
_queued = []


def fake_parse(metadata):
    return _queued.pop(0)


camera.parse_people = fake_parse

# A real inference result publishes and timestamps.
_queued.append([FakePerson(320, 240, 40000)])
camera._on_frame(req)
ts_good, persons_good = camera.get_latest()
check("real inference result is published",
      persons_good is not None and len(persons_good) == 1,
      "-> %r" % (persons_good,))

# A frame WITHOUT an inference result must leave the good one in place and must
# NOT advance its timestamp -- otherwise a stale detection would look fresh.
_queued.append(None)
camera._on_frame(req)
ts_after, persons_after = camera.get_latest()
check("inference-less frame does not clobber the last good detection",
      persons_after is persons_good, "-> %r" % (persons_after,))
check("inference-less frame does not refresh the timestamp",
      ts_after == ts_good, "ts %.6f vs %.6f" % (ts_after, ts_good))

# An empty list is a REAL result ("nobody here") and must replace the previous.
_queued.append([])
camera._on_frame(req)
ts_empty, persons_empty = camera.get_latest()
check("empty detection list replaces the previous result",
      persons_empty == [], "-> %r" % (persons_empty,))
check("empty result does refresh the timestamp", ts_empty > ts_good)

# A parse exception must not propagate into picamera2's thread, and must not
# publish anything.
def raising_parse(metadata):
    raise ValueError("boom")


camera.parse_people = raising_parse
errors_before = camera.health()["parse_errors"]
camera._on_frame(req)          # must not raise
check("parse exception is contained, not raised into the camera thread", True)
check("parse exception is counted",
      camera.health()["parse_errors"] == errors_before + 1)
ts_err, persons_err = camera.get_latest()
check("parse exception does not publish or refresh",
      persons_err == persons_empty and ts_err == ts_empty)

check("frame_age tracks callbacks even when inference fails",
      camera.frame_age() < 1.0, "-> %.3f s" % camera.frame_age())

camera.parse_people = _real_parse

raise SystemExit(summary())
