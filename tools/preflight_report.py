#!/usr/bin/env python3
"""BILBO pre-flight acceptance report.

Run this on the Pi, with the real hardware attached and the PROPELLERS OFF, on
the morning of a flight. It exercises every module, hammers the control path
with deliberately absurd inputs, measures the live sensors, and writes one CSV
row per check.

    python tools/preflight_report.py

Output: ~/bilbo-logs/preflight-<timestamp>.csv  plus a summary on the terminal.

SAFETY: this tool never commands motion. Adversarial velocity values are tested
against a capture shim, so nothing absurd reaches the wire. The only thing it
actually transmits is a single zero-velocity setpoint to prove the path works,
and it refuses to do even that while the vehicle is armed. It never spins a
motor -- that is `bench.py motors`, deliberately separate and separately gated.

Flags:
    --duration N     seconds to sample each live sensor (default 10)
    --no-mavlink     skip the Pixhawk section
    --no-lidar       skip the LiDAR section
    --no-camera      skip the camera section
    --no-led         skip the LED section
    --no-suite       skip running the offline test suite
"""

import argparse
import csv
import os
import platform
import statistics
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(REPO, "detectors"))

CRITICAL = "CRITICAL"   # a failure here blocks the flight
IMPORTANT = "IMPORTANT"  # a failure here is a strong warning
INFO = "INFO"           # recorded for the log, never blocks


class Report:
    """Accumulates rows, prints as it goes, writes one CSV at the end."""

    COLUMNS = ["section", "severity", "check", "result",
               "measured", "expected", "detail"]

    def __init__(self):
        self.rows = []
        self.section = "-"

    def begin(self, name):
        self.section = name
        print("\n" + "=" * 78)
        print(name)
        print("=" * 78)

    def add(self, severity, check, ok, measured="", expected="", detail=""):
        result = "PASS" if ok else "FAIL"
        self.rows.append({
            "section": self.section, "severity": severity, "check": check,
            "result": result, "measured": str(measured),
            "expected": str(expected), "detail": str(detail),
        })
        mark = "  ok  " if ok else "*FAIL*"
        line = "%s [%-9s] %s" % (mark, severity, check)
        if measured != "":
            line += "   -> %s" % measured
        if not ok and expected != "":
            line += "   (expected %s)" % expected
        print(line)
        return ok

    def note(self, check, value, detail=""):
        """An observation with no pass/fail meaning."""
        self.rows.append({
            "section": self.section, "severity": INFO, "check": check,
            "result": "INFO", "measured": str(value), "expected": "",
            "detail": str(detail),
        })
        print("  --   [INFO     ] %s   -> %s" % (check, value))

    def skip(self, check, why):
        self.rows.append({
            "section": self.section, "severity": INFO, "check": check,
            "result": "SKIP", "measured": "", "expected": "",
            "detail": str(why),
        })
        print("  --   [SKIP     ] %s   (%s)" % (check, why))

    def failures(self, severity):
        return [r for r in self.rows
                if r["result"] == "FAIL" and r["severity"] == severity]

    def write(self, path):
        with open(path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=self.COLUMNS)
            writer.writeheader()
            writer.writerows(self.rows)


R = Report()


def finite(x):
    try:
        return x == x and abs(x) != float("inf")
    except Exception:
        return False


# ============================================================ environment ===

def section_environment():
    R.begin("A. ENVIRONMENT")
    R.note("timestamp", time.strftime("%Y-%m-%d %H:%M:%S"))
    R.note("platform", platform.platform())
    R.note("python", sys.version.split()[0])
    R.note("repo", REPO)

    for name in ("pymavlink", "serial", "simple_pid", "pi5neo",
                 "picamera2", "cv2", "numpy"):
        try:
            mod = __import__(name)
            version = getattr(mod, "__version__", "installed")
            R.add(CRITICAL if name in ("pymavlink", "serial", "simple_pid")
                  else IMPORTANT,
                  "package %s importable" % name, True, version)
        except Exception as exc:
            R.add(CRITICAL if name in ("pymavlink", "serial", "simple_pid")
                  else IMPORTANT,
                  "package %s importable" % name, False,
                  "MISSING", "installed", repr(exc))

    try:
        import glob as _glob
        byid = sorted(_glob.glob("/dev/serial/by-id/*"))
        R.note("serial devices by-id", "; ".join(byid) or "none")
    except Exception:
        pass


# ================================================== offline logic suite ====

def section_suite():
    R.begin("B. OFFLINE LOGIC SUITE (hardware stubbed)")
    runner = os.path.join(REPO, "tests", "run_tests.py")
    if not os.path.exists(runner):
        R.skip("test suite present", "tests/run_tests.py not found")
        return
    try:
        proc = subprocess.run([sys.executable, runner],
                              capture_output=True, text=True, timeout=300)
    except Exception as exc:
        R.add(CRITICAL, "offline suite runs", False, "error", "exit 0",
              repr(exc))
        return

    out = (proc.stdout or "") + (proc.stderr or "")
    passed = sum(int(w.split()[0]) for w in
                 [l for l in out.splitlines() if "passed," in l]) \
        if "passed," in out else 0
    failed = 0
    for line in out.splitlines():
        if "passed," in line and "failed" in line:
            try:
                failed += int(line.split("passed,")[1].split()[0])
            except Exception:
                pass
    R.add(CRITICAL, "offline suite exits clean", proc.returncode == 0,
          "exit %d, %d checks passed, %d failed"
          % (proc.returncode, passed, failed), "exit 0")
    R.add(CRITICAL, "static cross-module audit clean",
          "no problems found" in out,
          "clean" if "no problems found" in out else "problems reported",
          "no problems found")
    for line in out.splitlines():
        if line.strip().startswith("FAIL"):
            R.note("suite failure detail", line.strip())


# ================================================== config consistency ====

def section_config():
    R.begin("C. CONFIGURATION CONSISTENCY")
    import config as cfg
    import controller
    import tracker

    for name in sorted(d for d in dir(cfg) if d.isupper()):
        R.note("cfg.%s" % name, getattr(cfg, name))

    R.add(CRITICAL, "setpoint interval faster than the control tick",
          cfg.MIN_SETPOINT_INTERVAL_S < cfg.TICK_PERIOD_S,
          "%.4f < %.4f" % (cfg.MIN_SETPOINT_INTERVAL_S, cfg.TICK_PERIOD_S))
    R.add(CRITICAL, "link timeout tolerates >=3 missed heartbeats at 1 Hz",
          cfg.LINK_TIMEOUT_S >= 3.0, "%.1f s" % cfg.LINK_TIMEOUT_S, ">= 3.0")
    R.add(CRITICAL, "tracker holds identity at least as long as the track",
          cfg.TRACK_MAX_MISSES * cfg.TICK_PERIOD_S >= cfg.LOST_TRACK_S - 1e-9,
          "%.2f s vs %.2f s"
          % (cfg.TRACK_MAX_MISSES * cfg.TICK_PERIOD_S, cfg.LOST_TRACK_S))

    worst, worst_at = float("-inf"), None
    for w in range(1, cfg.IMAGE_WIDTH_PX + 1):
        margin = controller.deadband_px(w) - tracker.hgate_px(w)
        if margin > worst:
            worst, worst_at = margin, w
    R.add(CRITICAL, "yaw deadband inside the range gate for every box width",
          worst < 0, "worst %+.1f px at width %d" % (worst, worst_at), "< 0")

    band_low = cfg.TARGET_DISTANCE_CM - cfg.DIST_DEADBAND_CM
    R.add(CRITICAL, "distance deadband clear of the safety floor",
          band_low > cfg.MIN_SAFE_DISTANCE_CM,
          "band bottom %.0f > floor %.0f cm"
          % (band_low, cfg.MIN_SAFE_DISTANCE_CM))
    R.add(CRITICAL, "hard clamps above the controller limits",
          cfg.HARD_MAX_SPEED_MS >= cfg.MAX_FORWARD_SPEED_MS
          and cfg.HARD_MAX_YAW_RATE_DEG_S >= cfg.MAX_YAW_RATE_DEG_S,
          "%.1f/%.0f vs %.1f/%.0f"
          % (cfg.HARD_MAX_SPEED_MS, cfg.HARD_MAX_YAW_RATE_DEG_S,
             cfg.MAX_FORWARD_SPEED_MS, cfg.MAX_YAW_RATE_DEG_S))
    R.add(CRITICAL, "search sweeps at least 360 deg before timing out",
          cfg.SEARCH_YAW_RATE_DEG_S * cfg.SEARCH_TIMEOUT_S >= 360.0,
          "%.0f deg" % (cfg.SEARCH_YAW_RATE_DEG_S * cfg.SEARCH_TIMEOUT_S))
    R.add(CRITICAL, "yaw sign is exactly +1 or -1",
          cfg.YAW_PID_OUTPUT_SIGN in (1.0, -1.0),
          "%+.1f" % cfg.YAW_PID_OUTPUT_SIGN)
    R.add(CRITICAL, "no gate-bypass attribute exists in config",
          not any("BENCH" in n.upper() or "BYPASS" in n.upper()
                  for n in dir(cfg)), "none found")
    R.add(IMPORTANT, "confidence threshold not so high it drops real people",
          cfg.CONF_THRESHOLD <= 0.55, "%.2f" % cfg.CONF_THRESHOLD, "<= 0.55")
    R.add(IMPORTANT, "MAVLINK_DEVICE has been set to a real path",
          "SET-ME" not in cfg.MAVLINK_DEVICE, cfg.MAVLINK_DEVICE,
          "a real /dev/serial/by-id/ path")


# ============================================ controller adversarial =======

CRAZY_ERRORS = [0.0, 1.0, -1.0, 12.5, -12.5, 49.0, 51.0, 320.0, -320.0,
                639.0, -639.0, 1e3, -1e3, 1e6, -1e6, 1e12, -1e12,
                float("nan"), float("inf"), float("-inf")]
CRAZY_DISTANCES = [None, 0.0, 1.0, -1.0, -1000.0, 19.0, 20.0, 150.0, 180.0,
                   200.0, 220.0, 400.0, 800.0, 801.0, 1e6, 1e12,
                   float("nan"), float("inf"), float("-inf")]
CRAZY_WIDTHS = [None, 0, -1, -1000, 1, 5, 120, 640, 10000, 1e9]


def section_controller():
    R.begin("D. CONTROLLER UNDER ABSURD INPUT (no transmission)")
    import config as cfg
    import controller

    bad = []
    exceptions = []
    n = 0
    worst_yaw = 0.0
    worst_fwd = 0.0

    for ex in CRAZY_ERRORS:
        for dist in CRAZY_DISTANCES:
            for width in CRAZY_WIDTHS:
                n += 1
                controller.reset()
                try:
                    t = controller.update(ex, dist, send=False,
                                          target_width_px=width,
                                          now=1000.0 + n * 0.0667)
                except Exception as exc:
                    exceptions.append((ex, dist, width, repr(exc)))
                    continue
                y, f = t["yaw_rate"], t["forward"]
                if not finite(y) or not finite(f):
                    bad.append((ex, dist, width, y, f, "non-finite"))
                    continue
                if abs(y) > cfg.MAX_YAW_RATE_DEG_S + 1e-6:
                    bad.append((ex, dist, width, y, f, "yaw over limit"))
                if abs(f) > cfg.MAX_FORWARD_SPEED_MS + 1e-6:
                    bad.append((ex, dist, width, y, f, "forward over limit"))
                worst_yaw = max(worst_yaw, abs(y))
                worst_fwd = max(worst_fwd, abs(f))

    R.note("input combinations exercised", n)
    R.add(CRITICAL, "controller never raises on absurd input",
          not exceptions, "%d exceptions" % len(exceptions), "0",
          "; ".join("%r" % (e,) for e in exceptions[:3]))
    R.add(CRITICAL, "controller output always finite and inside its limits",
          not bad, "%d violations" % len(bad), "0",
          "; ".join("%r" % (b,) for b in bad[:3]))
    R.note("worst |yaw| observed", "%.3f deg/s (limit %.0f)"
           % (worst_yaw, cfg.MAX_YAW_RATE_DEG_S))
    R.note("worst |forward| observed", "%.3f m/s (limit %.1f)"
           % (worst_fwd, cfg.MAX_FORWARD_SPEED_MS))

    # Sign convention, the single most safety-critical piece of maths here.
    controller.reset()
    right = controller.update(+120.0, None, send=False, now=2000.0)
    controller.reset()
    left = controller.update(-120.0, None, send=False, now=3000.0)
    R.add(CRITICAL, "target RIGHT commands POSITIVE yaw (nose right)",
          right["yaw_rate"] > 0, "%+.2f deg/s" % right["yaw_rate"], "> 0")
    R.add(CRITICAL, "target LEFT commands NEGATIVE yaw (nose left)",
          left["yaw_rate"] < 0, "%+.2f deg/s" % left["yaw_rate"], "< 0")

    # Distance sign and the deadband.
    def settle(d, ticks=90):
        controller.reset()
        t = None
        for i in range(ticks):
            t = controller.update(0.0, d, send=False,
                                  now=5000.0 + i * cfg.TICK_PERIOD_S)
        return t

    far = settle(cfg.TARGET_DISTANCE_CM + cfg.DIST_DEADBAND_CM + 80)
    near = settle(cfg.TARGET_DISTANCE_CM - cfg.DIST_DEADBAND_CM - 20)
    mid = settle(cfg.TARGET_DISTANCE_CM)
    R.add(CRITICAL, "too far commands FORWARD (positive)",
          far["forward"] > 0, "%+.3f m/s" % far["forward"], "> 0")
    R.add(CRITICAL, "too close commands BACKWARD (negative)",
          near["forward"] < 0, "%+.3f m/s" % near["forward"], "< 0")
    R.add(CRITICAL, "at the setpoint commands nothing",
          abs(mid["forward"]) < 1e-6, "%+.4f m/s" % mid["forward"], "0")

    for probe in (cfg.TARGET_DISTANCE_CM - cfg.DIST_DEADBAND_CM + 1,
                  cfg.TARGET_DISTANCE_CM + cfg.DIST_DEADBAND_CM - 1):
        t = settle(probe)
        R.add(IMPORTANT, "inside the distance deadband at %.0f cm: no output"
              % probe, abs(t["forward"]) < 1e-6,
              "%+.4f m/s" % t["forward"], "0")

    # Gating.
    t = settle(None)
    R.add(CRITICAL, "no range -> zero forward",
          t["forward"] == 0.0 and t["gate"] == "no_distance",
          "%+.3f (%s)" % (t["forward"], t["gate"]))
    controller.reset()
    t = controller.update(400.0, cfg.TARGET_DISTANCE_CM + 200, send=False,
                          now=7000.0)
    R.add(CRITICAL, "badly misaligned -> zero forward",
          t["forward"] == 0.0 and t["gate"] == "misaligned",
          "%+.3f (%s)" % (t["forward"], t["gate"]))
    t = settle(cfg.MIN_SAFE_DISTANCE_CM - 30)
    R.add(CRITICAL, "inside the safety floor -> backing off",
          t["backing_off"] and t["forward"] < 0,
          "%+.3f m/s, backing_off=%s" % (t["forward"], t["backing_off"]))

    # Slew: no single tick may jump more than the budget, including reversals.
    budget = cfg.FORWARD_SLEW_MS2 * cfg.TICK_PERIOD_S
    controller.reset()
    prev, worst_step = 0.0, 0.0
    seq = ([cfg.MIN_SAFE_DISTANCE_CM - 40] * 40
           + [cfg.TARGET_DISTANCE_CM + 400] * 40
           + [cfg.MIN_SAFE_DISTANCE_CM - 40] * 40)
    for i, d in enumerate(seq):
        t = controller.update(0.0, float(d), send=False,
                              now=9000.0 + i * cfg.TICK_PERIOD_S)
        worst_step = max(worst_step, abs(t["forward"] - prev))
        prev = t["forward"]
    R.add(CRITICAL, "forward slew respected across full sign reversals",
          worst_step <= budget + 1e-9,
          "worst step %.4f m/s" % worst_step, "<= %.4f" % budget)


# =============================================== tracker adversarial ======

def section_tracker():
    R.begin("E. TRACKER UNDER ABSURD INPUT")
    import config as cfg
    import tracker

    class P:
        def __init__(self, cx, cy, w, h):
            self.center_x, self.center_y = float(cx), float(cy)
            self.width, self.height = int(w), int(h)
            self.area = self.width * self.height
            self.x, self.y = int(cx - w / 2), int(cy - h / 2)

    try:
        tracker.configure(cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX)
        R.add(CRITICAL, "tracker geometry matches config", True,
              "%dx%d" % (cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX))
    except Exception as exc:
        R.add(CRITICAL, "tracker geometry matches config", False,
              "mismatch", "match", repr(exc))

    cases = [
        ("empty list", []),
        ("one normal person", [P(320, 240, 120, 340)]),
        ("zero-size box", [P(320, 240, 0, 0)]),
        ("box bigger than frame", [P(320, 240, 5000, 5000)]),
        ("box off the left edge", [P(-500, 240, 100, 300)]),
        ("box off the right edge", [P(5000, 240, 100, 300)]),
        ("twenty identical boxes", [P(320, 240, 100, 300)] * 20),
        ("fifty scattered boxes",
         [P(10 * i, 50 + 5 * i, 40 + i, 100 + i) for i in range(50)]),
    ]
    errors = []
    for label, persons in cases:
        try:
            tracker.reset()
            picked = tracker.select(persons)
            if picked is not None:
                lc = tracker.get_lock_center(200.0)
                locked, ex, ey = tracker.is_locked(picked, lc)
                if not (finite(ex) and finite(ey)):
                    errors.append((label, "non-finite error"))
            R.add(IMPORTANT, "select() survives: %s" % label, True,
                  "picked" if picked is not None else "None")
        except Exception as exc:
            errors.append((label, repr(exc)))
            R.add(CRITICAL, "select() survives: %s" % label, False,
                  "raised", "no exception", repr(exc))

    R.add(CRITICAL, "no tracker exceptions across all shapes", not errors,
          "%d errors" % len(errors), "0", "; ".join(str(e) for e in errors[:3]))

    for d in (None, 0.0, -50.0, 1.0, 200.0, 1e9, float("nan"),
              float("inf")):
        try:
            lc = tracker.get_lock_center(d)
            ok = finite(lc) and -1e4 < lc < 1e4
        except Exception:
            ok = False
            lc = "raised"
        R.add(CRITICAL, "get_lock_center sane at distance=%r" % d, ok, lc)

    # Gates must be monotone and clamped.
    gates = [tracker.hgate_px(w) for w in (1, 50, 120, 300, 10000)]
    R.add(CRITICAL, "horizontal gate is non-decreasing with box width",
          all(a <= b for a, b in zip(gates, gates[1:])), gates)
    R.add(CRITICAL, "horizontal gate clamped at both ends",
          tracker.hgate_px(1) == cfg.LIDAR_HGATE_MIN_PX
          and tracker.hgate_px(1e9) == cfg.LIDAR_HGATE_MAX_PX,
          "%.0f .. %.0f" % (tracker.hgate_px(1), tracker.hgate_px(1e9)))

    # Self-heal: a target that teleports must be re-acquired, not lost forever.
    tracker.reset()
    tracker.select([P(100, 240, 100, 300)])
    recovered_at = None
    for i in range(cfg.TRACK_MAX_MISSES + 4):
        got = tracker.select([P(600, 240, 100, 300)])
        if got is not None and recovered_at is None:
            recovered_at = i + 1
    R.add(CRITICAL, "tracker re-designates after a teleport (no permanent loss)",
          recovered_at is not None,
          "recovered on frame %s" % recovered_at,
          "<= %d" % (cfg.TRACK_MAX_MISSES + 1))


# ==================================================== LiDAR live sample ====

def section_lidar(duration):
    R.begin("F. LIDAR (live, %.0f s sample)" % duration)
    import config as cfg
    import lidar

    try:
        lidar.init()
        R.add(CRITICAL, "LiDAR port opens", True, cfg.LIDAR_DEVICE)
    except Exception as exc:
        R.add(CRITICAL, "LiDAR port opens", False, "failed", cfg.LIDAR_DEVICE,
              repr(exc))
        return

    print("\n  >>> AIM THE LIDAR AT A FLAT WALL ABOUT 2 m AWAY and hold the")
    print("  >>> airframe still for the next %.0f seconds. <<<\n"
          % duration)
    samples, gaps = [], 0
    strengths = []
    last_ok = None
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            d = lidar.read_data()
            now = time.monotonic()
            if d is not None:
                samples.append(d)
                s = lidar.last_strength()
                if s is not None:
                    strengths.append(s)
                if last_ok is not None and now - last_ok > 0.5:
                    gaps += 1
                last_ok = now
            time.sleep(1.0 / 30.0)
    finally:
        health = lidar.health()
        lidar.close()

    rate = len(samples) / duration
    R.note("valid readings", "%d (%.1f/s)" % (len(samples), rate))
    R.note("checksum failures", health["bad_checksum"])
    R.note("rejected (range/strength)", health["rejected"])
    R.note("dropouts longer than 0.5 s", gaps)

    R.add(CRITICAL, "LiDAR produces readings at a usable rate", rate >= 5.0,
          "%.1f/s" % rate, ">= 5/s")
    R.add(IMPORTANT, "checksum failures are rare",
          health["bad_checksum"] <= max(2, len(samples) * 0.02),
          health["bad_checksum"], "<= 2%% of frames",
          "a rising count means electrical noise on the UART")
    if samples:
        R.note("range min/median/max", "%d / %d / %d cm"
               % (min(samples), int(statistics.median(samples)), max(samples)))
        R.add(CRITICAL, "all readings inside the configured window",
              all(cfg.LIDAR_MIN_CM <= s <= cfg.LIDAR_MAX_CM for s in samples),
              "%d..%d cm" % (min(samples), max(samples)),
              "%d..%d" % (cfg.LIDAR_MIN_CM, cfg.LIDAR_MAX_CM))
        if len(samples) > 4:
            # Sample-to-sample jitter, NOT the spread over the whole run.
            # A std dev across the run conflates real sensor noise with
            # "the operator moved what it was aimed at" -- a 10 s sweep
            # from 34 cm to 276 cm reports 58 cm of "noise" when the
            # sensor was actually rock steady. Consecutive differences
            # are immune to slow changes in the scene and are what
            # actually drives micro-corrections.
            steps = [abs(b - a) for a, b in zip(samples, samples[1:])]
            jitter = statistics.median(steps)
            worst_step = max(steps)
            R.note("range spread over the whole sample",
                   "%.1f cm std dev (scene-dependent, not a fault)"
                   % statistics.pstdev(samples))
            R.note("range jitter (median consecutive change)",
                   "%.1f cm" % jitter)
            R.note("largest single-sample change", "%.0f cm" % worst_step)
            R.add(IMPORTANT, "sample-to-sample jitter well inside the distance deadband",
                  jitter < cfg.DIST_DEADBAND_CM / 2.0,
                  "%.1f cm" % jitter,
                  "< %.0f cm" % (cfg.DIST_DEADBAND_CM / 2.0),
                  "jitter near the deadband means constant micro-corrections")
            R.add(IMPORTANT, "no single jump beyond the jump-rejection threshold",
                  worst_step <= cfg.LIDAR_MAX_JUMP_CM,
                  "%.0f cm" % worst_step,
                  "<= %.0f cm" % cfg.LIDAR_MAX_JUMP_CM,
                  "larger jumps are rejected unless they repeat, which is intended")
    if strengths:
        R.note("signal strength min/median/max", "%d / %d / %d"
               % (min(strengths), int(statistics.median(strengths)),
                  max(strengths)))
        R.add(IMPORTANT, "signal strength comfortably above the floor",
              min(strengths) > cfg.LIDAR_MIN_STRENGTH,
              min(strengths), "> %d" % cfg.LIDAR_MIN_STRENGTH)
    else:
        R.add(CRITICAL, "any valid LiDAR reading at all", False, "none",
              ">0", "aim it at a wall 1-3 m away and re-run")


# =================================================== camera live sample ===

def section_camera(duration):
    R.begin("G. CAMERA / DETECTION (live, %.0f s sample)" % duration)
    import config as cfg
    import camera
    import tracker

    try:
        w, h = camera.intitalise()
        R.add(CRITICAL, "camera starts", True, "%dx%d" % (w, h))
    except Exception as exc:
        R.add(CRITICAL, "camera starts", False, "failed", "started", repr(exc))
        return

    try:
        R.add(CRITICAL, "camera geometry matches config",
              w == cfg.IMAGE_WIDTH_PX and h == cfg.IMAGE_HEIGHT_PX,
              "%dx%d" % (w, h),
              "%dx%d" % (cfg.IMAGE_WIDTH_PX, cfg.IMAGE_HEIGHT_PX))
        tracker.configure(w, h)
    except Exception as exc:
        R.add(CRITICAL, "camera geometry matches config", False, "mismatch",
              "match", repr(exc))

    # Wait for the first real inference before timing anything. camera._latest
    # starts as (0.0, None), so sampling immediately measures an "age" of
    # now - 0.0 -- the Pi uptime, which reported as 4181361 ms and failed
    # the staleness check for no reason.
    warm = time.monotonic() + 10.0
    while not camera.inference_alive() and time.monotonic() < warm:
        time.sleep(0.05)
    R.add(CRITICAL, "inference started within 10 s",
          camera.inference_alive(),
          "yes" if camera.inference_alive() else "no")

    print("\n  >>> STAND ABOUT 2 m FROM THE CAMERA -- not at arm's length --")
    print("  >>> and walk fully LEFT then fully RIGHT across the frame,")
    print("  >>> for the next %.0f seconds. <<<\n" % duration)

    deadline = time.monotonic() + duration
    frames = 0
    with_person = 0
    ages = []
    widths = []
    errors_x = []
    tracked = 0
    assoc_fail = 0
    last_ts = None
    try:
        while time.monotonic() < deadline:
            ts, persons = camera.get_latest()
            if ts != last_ts and ts > 0.0:
                frames += 1
                last_ts = ts
                ages.append(time.monotonic() - ts)
                if persons:
                    with_person += 1
                    target = tracker.select(persons)
                    if target is None:
                        assoc_fail += 1
                    else:
                        tracked += 1
                        widths.append(target.width)
                        lc = tracker.get_lock_center(
                            cfg.TARGET_DISTANCE_CM)
                        _, ex, _ = tracker.is_locked(target, lc)
                        errors_x.append(ex)
            time.sleep(1.0 / 40.0)
    finally:
        health = camera.health()
        camera.stop()

    fps = frames / duration
    R.note("published frames", "%d (%.1f/s)" % (frames, fps))
    R.note("inference outputs", health.get("inferences"))
    R.note("parse errors", health.get("parse_errors"))
    R.note("frames containing a person", with_person)
    R.note("frames with a tracked target", tracked)
    R.note("association failures", assoc_fail)

    R.add(CRITICAL, "inference actually produced output",
          health.get("inferences", 0) > 0, health.get("inferences"), "> 0")
    R.add(CRITICAL, "no parse errors", health.get("parse_errors", 0) == 0,
          health.get("parse_errors"), "0")
    R.add(CRITICAL, "detection rate high enough for the control loop",
          fps >= cfg.TICK_HZ * 0.6, "%.1f/s" % fps,
          ">= %.1f/s" % (cfg.TICK_HZ * 0.6))
    if ages:
        R.note("detection age min/median/max", "%.0f / %.0f / %.0f ms"
               % (min(ages) * 1000, statistics.median(ages) * 1000,
                  max(ages) * 1000))
        R.add(CRITICAL, "detections stay fresher than the staleness limit",
              max(ages) < cfg.VISION_MAX_AGE_S,
              "%.0f ms worst" % (max(ages) * 1000),
              "< %.0f ms" % (cfg.VISION_MAX_AGE_S * 1000))
    R.add(CRITICAL, "a person was detected during the sample",
          with_person > 0, with_person, "> 0",
          "if zero: check lighting, and try --duration 20")
    if with_person:
        detect_ratio = with_person / float(frames or 1)
        R.add(IMPORTANT, "person present in most frames while standing there",
              detect_ratio > 0.5, "%.0f%%" % (100 * detect_ratio), "> 50%",
              "low means CONF_THRESHOLD is still too high, or poor lighting")
        R.add(IMPORTANT, "association rarely fails while tracking",
              assoc_fail <= max(2, with_person * 0.25), assoc_fail,
              "<= 25%% of frames",
              "high means TRACK_GATE_PX is too tight")
    if widths:
        R.note("target box width min/median/max", "%d / %d / %d px"
               % (min(widths), int(statistics.median(widths)), max(widths)))
        import tracker as _t
        median_w = statistics.median(widths)
        R.note("resulting range gate at median width",
               "%.0f px" % _t.hgate_px(median_w))
        # Box width is a usable range estimate: a ~0.5 m shoulder width
        # subtending median_w pixels puts the subject at this distance.
        import math as _m
        arc = median_w / cfg.PIXELS_PER_DEGREE
        est = (0.25 / _m.tan(_m.radians(arc / 2.0))
               if 0 < arc < 179 else float("inf"))
        R.note("estimated subject distance from box width",
               "%.2f m (%.0f deg of arc)" % (est, arc))
        R.add(IMPORTANT, "subject stood at a realistic tracking distance",
              0.8 < est < 8.0, "%.2f m" % est, "0.8 - 8 m",
              "at arm's length the box fills the frame, error_x cannot move, "
              "and the range gate sits clamped at its ceiling -- the sample "
              "does not represent flight")
    if errors_x:
        R.note("error_x range observed", "%+.0f .. %+.0f px"
               % (min(errors_x), max(errors_x)))
        R.add(IMPORTANT, "target was seen on BOTH sides of centre",
              min(errors_x) < -10 and max(errors_x) > 10,
              "%+.0f .. %+.0f px" % (min(errors_x), max(errors_x)),
              "one reading below -10 and one above +10",
              "stand further back and walk fully to both edges of frame; this "
              "is what exercises the yaw sign in both directions")


# ================================================== MAVLink live sample ===

def section_mavlink(duration):
    R.begin("H. MAVLINK LINK (live, %.0f s sample)" % duration)
    import config as cfg
    import drone

    if not drone.connect(timeout=10.0):
        R.add(CRITICAL, "Pixhawk heartbeat received", False, "no link",
              "connected",
              "check MAVLINK_DEVICE, SERIALn_BAUD/PROTOCOL, and "
              "BRD_SERn_RTSCTS=0 for a 3-wire FTDI")
        return

    R.add(CRITICAL, "Pixhawk heartbeat received", True,
          "sys %d comp %d" % (drone.master.target_system,
                              drone.master.target_component))
    R.note("configured baud", cfg.MAVLINK_BAUD)

    seen, bad_data = {}, 0
    hb_times = []
    start = time.monotonic()
    while time.monotonic() - start < duration:
        msg = drone.master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        name = msg.get_type()
        if name == "BAD_DATA":
            bad_data += 1
            continue
        key = (msg.get_srcSystem(), msg.get_srcComponent(), name)
        seen[key] = seen.get(key, 0) + 1
        if name == "HEARTBEAT":
            hb_times.append(time.monotonic())
        drone._ingest(msg)
    elapsed = time.monotonic() - start

    for key in sorted(seen):
        R.note("rate sys%s comp%s %s" % key, "%.1f Hz (%d)"
               % (seen[key] / elapsed, seen[key]))

    nodes = sorted(set((k[0], k[1]) for k in seen))
    R.note("MAVLink nodes on this link", nodes)
    R.note("messages ignored as foreign", drone.foreign_msgs())
    if len(nodes) > 1:
        R.note("multiple nodes present",
               "normal with a GCS or telemetry radio; the Pi filters them")

    # Link budget. 8N1 means 10 bits per byte, so the byte budget is
    # baud/10. Oversubscription shows up as requested stream rates not
    # being honoured and as a draining backlog (heartbeat intervals of
    # 0.00 s), both of which add latency to everything the Pi reads.
    APPROX_BYTES = {"HEARTBEAT": 23, "COMMAND_ACK": 24}
    est_bytes = sum(seen[k] * APPROX_BYTES.get(k[2], 40) for k in seen)
    est_rate = est_bytes / elapsed
    budget = cfg.MAVLINK_BAUD / 10.0
    R.note("estimated inbound traffic", "%.0f B/s" % est_rate)
    R.note("link byte budget at %d baud" % cfg.MAVLINK_BAUD,
           "%.0f B/s" % budget)
    R.add(IMPORTANT, "inbound traffic within the link budget",
          est_rate < budget * 0.8,
          "%.0f B/s = %.0f%% of budget" % (est_rate,
                                          100.0 * est_rate / budget),
          "< 80%% of %.0f B/s" % budget,
          "raise the Pi link to 921600 (SERIALn_BAUD=921 and cfg.MAVLINK_BAUD), "
          "and/or disconnect Mission Planner: a GCS on the telemetry radio makes "
          "ArduPilot route its traffic onto this link too")

    rc_rate = sum(seen[k] for k in seen if k[2] == "RC_CHANNELS") / elapsed
    R.note("RC_CHANNELS rate", "%.1f Hz" % rc_rate)
    R.add(CRITICAL, "RC_CHANNELS arrives faster than the staleness timeout",
          rc_rate > 1.0 / cfg.RC_STALE_S * 2.0,
          "%.1f Hz" % rc_rate,
          "> %.1f Hz" % (2.0 / cfg.RC_STALE_S),
          "below this the AI switch can read stale and fail closed mid-flight")

    hb_rate = sum(seen[k] for k in seen
                  if k[2] == "HEARTBEAT" and k[0] == 1) / elapsed
    R.note("autopilot HEARTBEAT rate", "%.1f Hz" % hb_rate)
    R.add(IMPORTANT, "heartbeat rate is plausible, not a drained backlog",
          hb_rate < 20.0, "%.1f Hz" % hb_rate, "< 20 Hz",
          "a rate this high means messages were queued and arrived in a burst, "
          "which is a saturated link -- everything the Pi reads is then late")

    present = set(k[2] for k in seen)
    R.add(CRITICAL, "HEARTBEAT streaming", "HEARTBEAT" in present,
          "yes" if "HEARTBEAT" in present else "no")
    R.add(CRITICAL, "RC_CHANNELS streaming", "RC_CHANNELS" in present,
          "yes" if "RC_CHANNELS" in present else "no",
          "required", "without it ch%d cannot be read and autonomy can "
          "never engage" % cfg.CH_ENABLE)
    R.add(IMPORTANT, "BAD_DATA frames rare", bad_data <= 5, bad_data, "<= 5",
          "a steady stream means a baud mismatch or noise")

    if len(hb_times) > 2:
        deltas = [b - a for a, b in zip(hb_times, hb_times[1:])]
        R.note("heartbeat interval median/max", "%.2f / %.2f s"
               % (statistics.median(deltas), max(deltas)))
        R.add(CRITICAL, "worst heartbeat gap well inside the link timeout",
              max(deltas) < cfg.LINK_TIMEOUT_S * 0.6,
              "%.2f s" % max(deltas),
              "< %.2f s" % (cfg.LINK_TIMEOUT_S * 0.6))

    R.note("flight mode", drone.get_mode())
    R.note("armed", drone.is_armed())
    alt = drone.get_relative_alt()
    R.note("relative altitude", "unknown" if alt is None else "%.2f m" % alt)

    rc = drone.get_rc_channels()
    R.note("RC channels seen", sorted(rc))
    for ch in sorted(rc):
        R.note("ch%d" % ch, rc[ch])
    enable_raw = rc.get(cfg.CH_ENABLE)
    R.add(CRITICAL, "ch%d (AI enable) is readable" % cfg.CH_ENABLE,
          enable_raw is not None, enable_raw, "a pulse value",
          "map a 2-position switch to ch%d on the transmitter"
          % cfg.CH_ENABLE)
    if enable_raw is not None:
        R.note("ch%d decodes as" % cfg.CH_ENABLE,
               "ON" if enable_raw >= cfg.SWITCH_ON_US else "OFF")

    # Adversarial setpoints against a capture shim: nothing goes on the wire.
    real_send = drone.master.mav.set_position_target_local_ned_send
    captured = []

    def capture(*args):
        captured.append(args)

    drone.master.mav.set_position_target_local_ned_send = capture
    try:
        violations = []
        for fs in (0.0, 1e9, -1e9, float("nan"), float("inf"),
                   float("-inf"), None):
            for yr in (0.0, 1e9, -1e9, float("nan"), float("inf"), None):
                drone._last_setpoint_tx = 0.0
                drone.move(fs, yaw_rate=yr, force=True)
                if not captured:
                    continue
                a = captured[-1]
                vx, vy, vz, yaw_rate_rad = a[8], a[9], a[10], a[15]
                for label, v, limit in (
                        ("vx", vx, cfg.HARD_MAX_SPEED_MS),
                        ("vy", vy, cfg.HARD_MAX_SPEED_MS),
                        ("vz", vz, cfg.HARD_MAX_SPEED_MS),
                        ("yaw", yaw_rate_rad, 100.0)):
                    if not finite(v) or abs(v) > limit + 1e-6:
                        violations.append((fs, yr, label, v))
        R.add(CRITICAL, "absurd velocities are clamped before transmission",
              not violations, "%d violations" % len(violations), "0",
              "; ".join(str(v) for v in violations[:3]))
        R.note("setpoints captured during clamp test", len(captured))
        if captured:
            R.add(CRITICAL, "frame is MAV_FRAME_BODY_NED (8)",
                  captured[-1][3] == 8, captured[-1][3], "8")
            R.add(CRITICAL, "bitmask is the velocity+yaw_rate mask (1479)",
                  captured[-1][4] == 1479, captured[-1][4], "1479")
    finally:
        drone.master.mav.set_position_target_local_ned_send = real_send

    # One real zero-velocity send, and only while disarmed.
    if drone.is_armed():
        R.skip("real zero-velocity setpoint accepted",
               "vehicle is ARMED -- refusing to transmit")
    else:
        drone._last_setpoint_tx = 0.0
        R.add(CRITICAL, "real zero-velocity setpoint transmits",
              drone.move(0.0, 0.0, 0.0, 0.0, force=True), "sent", "sent",
              "inert: ArduPilot ignores guided setpoints outside GUIDED")

    R.note("mode change capability", "not tested -- would move the aircraft")
    drone.disconnect_drone()


# ======================================================== LED live test ===

def section_led():
    R.begin("I. LED STRIP")
    import config as cfg
    import led
    from state import DroneState

    if not led.init():
        R.add(IMPORTANT, "LED strip opens", False, "unavailable",
              cfg.LED_DEVICE,
              "check SPI enabled and the user is in the spi group")
        return
    R.add(IMPORTANT, "LED strip opens", True, cfg.LED_DEVICE)
    R.note("brightness handled in", "hardware" if led._hw_brightness
           else "software")
    R.note("fill_strip convention", "tuple" if led._fill_takes_tuple
           else "r,g,b")

    R.add(CRITICAL, "every state has a colour mapping",
          all(s in led._STATE_APPEARANCE for s in DroneState),
          "%d of %d" % (len(led._STATE_APPEARANCE), len(list(DroneState))))
    errors = []
    for st in DroneState:
        try:
            led.render(st)
            led.render(st, fault="test")
        except Exception as exc:
            errors.append((st.name, repr(exc)))
    R.add(CRITICAL, "no state raises while rendering", not errors,
          "%d errors" % len(errors), "0", str(errors[:2]))

    print("\n  >>> WATCH THE STRIP: red, green, blue, then off. <<<")
    led.selftest()
    R.note("visual self-test", "run -- confirm you saw R, G, B")
    led.shutdown()


# ============================================================== verdict ===

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--no-mavlink", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--no-led", action="store_true")
    ap.add_argument("--no-suite", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    print(__doc__.split("Flags:")[0])
    started = time.time()

    sections = [
        (True, section_environment, ()),
        (not args.no_suite, section_suite, ()),
        (True, section_config, ()),
        (True, section_controller, ()),
        (True, section_tracker, ()),
        (not args.no_lidar, section_lidar, (args.duration,)),
        (not args.no_camera, section_camera, (args.duration,)),
        (not args.no_mavlink, section_mavlink, (args.duration,)),
        (not args.no_led, section_led, ()),
    ]
    for enabled, fn, fnargs in sections:
        if not enabled:
            R.begin(fn.__name__.replace("section_", "").upper())
            R.skip(fn.__name__, "disabled by flag")
            continue
        try:
            fn(*fnargs)
        except Exception:
            import traceback
            R.add(CRITICAL, "%s completed" % fn.__name__, False, "crashed",
                  "completed", traceback.format_exc().replace("\n", " | "))

    # ---- write the CSV -----------------------------------------------------
    try:
        import config as cfg
        outdir = cfg.LOG_DIR
    except Exception:
        outdir = os.path.expanduser("~")
    try:
        os.makedirs(outdir, exist_ok=True)
    except Exception:
        outdir = os.path.expanduser("~")
    path = args.out or os.path.join(
        outdir, "preflight-%d.csv" % int(started))
    try:
        R.write(path)
    except Exception as exc:
        print("\ncould not write CSV: %r" % exc)
        path = None

    # ---- verdict -----------------------------------------------------------
    crit = R.failures(CRITICAL)
    imp = R.failures(IMPORTANT)
    total = len(R.rows)
    passed = len([r for r in R.rows if r["result"] == "PASS"])

    print("\n" + "=" * 78)
    print("VERDICT")
    print("=" * 78)
    print("  %d rows recorded, %d passed, %d critical failures, "
          "%d important failures" % (total, passed, len(crit), len(imp)))
    print("  elapsed %.0f s" % (time.time() - started))
    if path:
        print("  report: %s" % path)

    if crit:
        print("\n  *** DO NOT FLY ***  %d critical check(s) failed:" % len(crit))
        for r in crit:
            print("      - [%s] %s" % (r["section"].split(".")[0], r["check"]))
            if r["detail"]:
                print("          %s" % r["detail"][:160])
    elif imp:
        print("\n  CLEARED WITH WARNINGS -- %d important check(s) failed:"
              % len(imp))
        for r in imp:
            print("      - [%s] %s -> %s"
                  % (r["section"].split(".")[0], r["check"], r["measured"]))
        print("\n  None of these block flight, but understand each one first.")
    else:
        print("\n  ALL CHECKS PASSED.")

    print("\n  Still not covered by this tool, and still mandatory:")
    print("    1. tools/verify_yaw_sign.py -- stand to the drone's RIGHT,")
    print("       confirm it reports image RIGHT and nose RIGHT. Props off.")
    print("    2. bench.py motors --throttle 15 -- motor order and direction.")
    print("    3. A switch mapped to ch8, confirmed moving in bench.py switch.")
    print("    4. GUIDED and RTL set on the flight-mode channel.")
    print("    5. In the air: GUIDED then straight back to LOITER, twice,")
    print("       BEFORE you ever touch ch8.")

    return 1 if crit else 0


if __name__ == "__main__":
    sys.exit(main())
