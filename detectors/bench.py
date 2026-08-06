#!/usr/bin/env python3
"""BILBO bench diagnostics -- ground testing only, propellers OFF.

A standalone tool for answering "is the hardware wired up and behaving?" before
any autonomous code is trusted. It shares the driver modules with the flight
code so that what you verify here is what will actually run, but it contains no
state machine, no autonomy and no engagement logic of its own.

    python detectors/bench.py link       # MAVLink: heartbeat, mode, RC, streams
    python detectors/bench.py switch     # live ch6 readout
    python detectors/bench.py sensors    # LiDAR + camera health, text only
    python detectors/bench.py leds       # walk through every LED state
    python detectors/bench.py track      # live tracking preview + commands
    python detectors/bench.py motors     # spin each motor briefly, PROPS OFF

`track` computes the yaw and forward commands and shows them, but sends
NOTHING to the flight controller unless you add --send.

`motors` uses ArduPilot's MAV_CMD_DO_MOTOR_TEST, the same mechanism as Mission
Planner's motor test screen. It spins motors while DISARMED, which is the
correct and lowest-risk way to check wiring and direction.
"""

import argparse
import logging
import sys
import time

from pymavlink import mavutil

import config as cfg
import controller
import drone
import lidar
import tracker

_log = logging.getLogger("bench")


# ------------------------------------------------------------------ utils ----

def setup_logging(verbose=False):
    logging.basicConfig(
        level=logging.DEBUG if verbose else logging.INFO,
        format="%(levelname)-7s %(name)s: %(message)s",
        stream=sys.stderr,
    )


def hr(title=""):
    print("\n" + "-" * 72)
    if title:
        print(title)
        print("-" * 72)


def connect_or_die(timeout=10.0):
    if not drone.connect(timeout=timeout):
        print("\nFAILED to connect to the Pixhawk.")
        print("  Check, in order:")
        print("   1. config.MAVLINK_DEVICE matches `ls -l /dev/serial/by-id/`")
        print("   2. config.MAVLINK_BAUD (%d) matches the Pixhawk's "
              "SERIALn_BAUD" % cfg.MAVLINK_BAUD)
        print("   3. SERIALn_PROTOCOL = 2 (MAVLink2) for the port you wired")
        print("   4. TX/RX are crossed, and grounds are common")
        raise SystemExit(1)


# ------------------------------------------------------------------- link ----

def cmd_link(args):
    """Verify the MAVLink connection end to end."""
    connect_or_die()

    hr("LINK")
    print("device   : %s" % cfg.MAVLINK_DEVICE)
    print("baud     : %d" % cfg.MAVLINK_BAUD)
    print("system   : %d   component: %d"
          % (drone.master.target_system, drone.master.target_component))

    # Which message types actually arrive, and at what rate. If RC_CHANNELS is
    # absent the AI-enable switch can never be read, so this is the single most
    # useful thing on the page.
    seen = {}
    start = time.monotonic()
    window = args.seconds

    print("\nlistening for %.0f s ..." % window)
    while time.monotonic() - start < window:
        msg = drone.master.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        name = msg.get_type()
        if name == "BAD_DATA":
            seen["BAD_DATA"] = seen.get("BAD_DATA", 0) + 1
            continue
        seen[name] = seen.get(name, 0) + 1
        drone._ingest(msg)

    elapsed = time.monotonic() - start

    hr("MESSAGE RATES")
    for name in sorted(seen):
        print("  %-28s %6.1f Hz  (%d)"
              % (name, seen[name] / elapsed, seen[name]))

    # Only these two are required for a bench test. GPS and altitude are
    # deliberately NOT required: indoors there is no fix, and nothing on this
    # bench needs a position estimate.
    required = {
        "HEARTBEAT": "mode, armed state and link liveness",
        "RC_CHANNELS": "ch%d AI-enable switch" % cfg.CH_ENABLE,
    }
    optional = {
        "GLOBAL_POSITION_INT": "relative altitude (outdoor flight only)",
        "ATTITUDE": "attitude (diagnostics only)",
    }

    hr("REQUIRED FOR BENCH TESTING")
    missing = []
    for name, why in required.items():
        ok = seen.get(name, 0) > 0
        print("  [%s] %-22s %s" % ("OK" if ok else "--", name, why))
        if not ok:
            missing.append(name)

    print("\nNot needed on the bench (absent indoors is expected):")
    for name, why in optional.items():
        ok = seen.get(name, 0) > 0
        print("  [%s] %-22s %s" % ("ok" if ok else "  ", name, why))

    if missing:
        print("\n  MISSING: %s" % ", ".join(missing))
        print("  This build requests streams with MAV_CMD_SET_MESSAGE_INTERVAL,")
        print("  so if RC_CHANNELS is absent set the stream rates explicitly")
        print("  for the port the FTDI is wired to and re-run:")
        print("    SRn_RC_CHAN = 10, SRn_EXT_STAT = 2")
        print("  If the whole link is silent, suspect flow control: a 3-wire")
        print("  FTDI needs BRD_SERn_RTSCTS = 0, not 2 (auto).")

    if seen.get("BAD_DATA"):
        print("\n  %d BAD_DATA frames. A steady stream of these means a baud "
              "mismatch\n  or electrical noise on the link."
              % seen["BAD_DATA"])

    hr("VEHICLE STATE")
    print("mode     : %s" % drone.get_mode())
    print("armed    : %s" % drone.is_armed())
    print("hb age   : %.2f s" % drone.link_age())

    rc = drone.get_rc_channels()
    if rc:
        print("\nRC channels (raw us):")
        for ch in sorted(rc):
            marker = "  <-- ch%d AI enable" % cfg.CH_ENABLE \
                if ch == cfg.CH_ENABLE else ""
            print("  ch%-2d %5s%s" % (ch, rc[ch], marker))
    else:
        print("\nNO RC DATA -- ch%d unreadable." % cfg.CH_ENABLE)

    # Setpoint acceptance. Zero velocity is inert but proves the message is
    # accepted and the mask/frame are formed correctly.
    hr("SETPOINT PATH")
    ok = drone.hover(force=True)
    print("zero-velocity setpoint sent: %s" % ("OK" if ok else "FAILED"))
    print("  frame  : MAV_FRAME_BODY_NED (%d)" % drone.VELOCITY_FRAME)
    print("  bitmask: %d (velocity + yaw_rate)" % drone.VELOCITY_YAW_MASK)
    print("\nNote: ArduPilot ignores guided setpoints outside GUIDED mode and")
    print("while disarmed, so nothing moves. That is the pilot-authority")
    print("mechanism working, not a failure.")

    return 0 if not missing else 1


def cmd_switch(args):
    """Live RC readout, for confirming the 3-position AI switch bands."""
    connect_or_die()
    print("Move the ch%d switch through all positions. Ctrl-C to stop.\n"
          % cfg.CH_ENABLE)
    print("  threshold: OFF below %d us, ON at or above it"
          % cfg.SWITCH_ON_US)
    try:
        while True:
            drone.poll()
            raw = drone.get_channel(cfg.CH_ENABLE)
            age = drone.rc_age()
            if raw is None:
                label = "NO RC DATA"
            elif age > cfg.RC_STALE_S:
                label = "STALE (%.1fs) -> treated as OFF" % age
            elif raw < cfg.SWITCH_ON_US:
                label = "OFF"
            else:
                label = "ON (full tracking)"
            print("\r  ch%d = %5s us   %-34s   mode=%-10s armed=%-5s"
                  % (cfg.CH_ENABLE, raw, label, drone.get_mode(),
                     drone.is_armed()), end="", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


# ---------------------------------------------------------------- sensors ----

def cmd_sensors(args):
    """LiDAR and camera health, no motion, no MAVLink required."""
    hr("LIDAR")
    try:
        lidar.init()
    except Exception as exc:
        print("FAILED to open %s: %s" % (cfg.LIDAR_DEVICE, exc))
        print("  'device or resource busy' means something else holds the "
              "port.")
        print("  Check it is not the same device as config.MAVLINK_DEVICE, and")
        print("  that the serial console is disabled (raspi-config).")
        return 1

    print("open on %s @ %d" % (cfg.LIDAR_DEVICE, cfg.LIDAR_BAUD))
    print("accepting %d-%d cm, min strength %d\n"
          % (cfg.LIDAR_MIN_CM, cfg.LIDAR_MAX_CM, cfg.LIDAR_MIN_STRENGTH))

    import camera
    cfg.DEBUG_PREVIEW = False
    try:
        width, height = camera.intitalise()
        tracker.configure(width, height)
        cam_ok = True
        print("camera %dx%d started" % (width, height))
    except Exception as exc:
        print("camera failed to start: %s" % exc)
        cam_ok = False

    print("\nCtrl-C to stop.\n")
    try:
        while True:
            distance = lidar.read_data()
            health = lidar.health()
            line = "lidar %s  strength %-6s  valid %-6d bad_crc %-4d rej %-4d" \
                % ("  --  " if distance is None else "%4d cm" % distance,
                   health["strength"], health["valid"],
                   health["bad_checksum"], health["rejected"])
            if cam_ok:
                ts, persons = camera.get_latest()
                age = time.monotonic() - ts
                line += "  |  vision %5.0f ms  persons %s" % (
                    age * 1000.0,
                    "?" if persons is None else len(persons))
            print("\r" + line, end="", flush=True)
            time.sleep(1.0 / 10.0)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        lidar.close()
        if cam_ok:
            camera.stop()
    return 0


# -------------------------------------------------------------------- LEDs ----

def cmd_leds(args):
    """Walk through every LED state so the strip and colours can be verified.

    Needs no Pixhawk, no GPS and no arming -- just the SPI wiring.
    """
    import led

    hr("LED CHECK")
    if not led.init():
        print("LED strip unavailable. Check:")
        print("  - SPI enabled (raspi-config -> Interface Options -> SPI)")
        print("  - %s exists" % cfg.LED_DEVICE)
        print("  - your user is in the 'spi' group (log out and back in)")
        print("  - pi5neo installed")
        return 1

    print("%d LEDs on %s, brightness %.2f"
          % (cfg.LED_COUNT, cfg.LED_DEVICE, cfg.LED_BRIGHTNESS))
    print("\nSelf-test: red, green, blue. Every LED should light each colour.")
    print("If colours look swapped, the strip is GRB rather than RGB.\n")
    led.selftest()

    from state import DroneState
    try:
        return _walk_led_states(args, DroneState)
    except KeyboardInterrupt:
        print("\nstopped")
        return 130
    finally:
        # Ctrl-C must not leave the strip lit.
        led.shutdown()


def _walk_led_states(args, DroneState):
    import led

    order = [
        (DroneState.IDLE, "not in GUIDED / booting"),
        (DroneState.READY, "in GUIDED, armed, waiting for the ch%d switch"
                           % cfg.CH_ENABLE),
        (DroneState.TRACKING, "locked onto a person"),
        (DroneState.SEARCHING, "target lost, sweeping"),
        (DroneState.RTL, "returning home"),
        (DroneState.LANDING, "landing"),
        (DroneState.EMERGENCY, "fault"),
    ]

    for state, meaning in order:
        effect, colour = led._STATE_APPEARANCE[state]
        print("  %-10s %-6s %-8s  %s"
              % (state.name, effect, colour, meaning))
        end = time.monotonic() + args.dwell
        while time.monotonic() < end:
            led.render(state)
            time.sleep(0.02)

    print("\nFault indication (blinking orange) -- outranks the state colour:")
    end = time.monotonic() + args.dwell
    while time.monotonic() < end:
        led.render(DroneState.TRACKING, fault="demo")
        time.sleep(0.02)

    print("\nLiveness blips: two quick white flashes every %.0f s while the"
          % led.LIVENESS_PERIOD_S)
    print("control loop is ticking. If these STOP in flight, the loop stalled.")
    end = time.monotonic() + max(args.dwell, led.LIVENESS_PERIOD_S * 2.2)
    while time.monotonic() < end:
        led.note_tick()                 # pretend the loop is healthy
        led.render(DroneState.TRACKING)
        time.sleep(0.02)

    print("\nNow WITHOUT ticking the loop -- blips must stop, colour holds:")
    led._last_tick_time = 0.0
    end = time.monotonic() + args.dwell
    while time.monotonic() < end:
        led.render(DroneState.TRACKING)
        time.sleep(0.02)

    hr("DONE")
    print("Strip blanked. If any state showed the wrong colour, the mapping is")
    print("in led._STATE_APPEARANCE.")
    return 0


# ------------------------------------------------------------ tracking UI ----

_overlay = {
    "target": None, "persons": [], "error_x": 0.0, "error_y": 0.0,
    "distance": None, "strength": None, "locked": False, "gate_open": False,
    "yaw_rate": 0.0, "forward": 0.0, "reason": "-", "fps": 0.0,
    "sending": False, "yaw_hint": "hold", "status": "NONE", "misses": 0,
    "deadband_px": cfg.YAW_DEADBAND_PX,
    "hgate_px": cfg.LIDAR_HGATE_MIN_PX,
    "vgate_px": cfg.LIDAR_VGATE_MIN_PX,
}

GREY = (140, 140, 140)
GREEN = (0, 255, 0)
CYAN = (255, 255, 0)
WHITE = (255, 255, 255)
RED = (0, 0, 255)
YELLOW = (0, 220, 220)


def _draw(request):
    """Annotate the preview with the tracking decision. Camera thread."""
    import cv2
    from picamera2 import MappedArray

    o = dict(_overlay)
    with MappedArray(request, "main") as m:
        img = m.array
        h, w = img.shape[0], img.shape[1]
        cx = w // 2

        # Reference geometry.
        cv2.line(img, (cx, 0), (cx, h), WHITE, 1)
        for sign in (-1, 1):
            db = int(cx + sign * o["deadband_px"])
            cv2.line(img, (db, 0), (db, h), GREY, 1)
            al = int(cx + sign * cfg.FORWARD_ALIGN_LIMIT_DEG
                     * cfg.PIXELS_PER_DEGREE)
            if 0 <= al < w:
                cv2.line(img, (al, 0), (al, h), YELLOW, 1)

        # LiDAR boresight row and its vertical gate.
        row = int(cfg.LIDAR_BORESIGHT_ROW_PX)
        cv2.line(img, (0, row), (w, row), CYAN, 1)
        for sign in (-1, 1):
            g = row + sign * int(o["vgate_px"])
            cv2.line(img, (0, g), (w, g), (90, 90, 0), 1)
        # Horizontal range gate, sized to the current target box.
        for sign in (-1, 1):
            hg = int(cx + sign * o["hgate_px"])
            if 0 <= hg < w:
                cv2.line(img, (hg, max(0, row - 40)),
                         (hg, min(h, row + 40)), CYAN, 1)

        # Every detection, thin. The chosen target, thick.
        for p in o["persons"]:
            cv2.rectangle(img, (p.x, p.y), (p.x + p.width, p.y + p.height),
                          GREY, 1)
        t = o["target"]
        if t is not None:
            colour = GREEN if o["gate_open"] else RED
            cv2.rectangle(img, (t.x, t.y), (t.x + t.width, t.y + t.height),
                          colour, 3)
            cv2.circle(img, (int(t.center_x), int(t.center_y)), 5, colour, -1)
            cv2.line(img, (cx, row), (int(t.center_x), int(t.center_y)),
                     colour, 1)

        lines = [
            "fps %.1f   %s" % (o["fps"],
                               "SENDING" if o["sending"] else "not sending"),
            "detections %-2d  target %s  misses %d"
            % (len(o["persons"]), o["status"], o["misses"]),
            "conf>=%.2f  deadband %.0f  hgate %.0f  vgate %.0f"
            % (cfg.CONF_THRESHOLD, o["deadband_px"], o["hgate_px"],
               o["vgate_px"]),
            "error_x %+7.1f px  (%+.1f deg)"
            % (o["error_x"], o["error_x"] / cfg.PIXELS_PER_DEGREE),
            "error_y %+7.1f px" % o["error_y"],
            "lidar   %s   want %d cm   strength %s"
            % ("----" if o["distance"] is None else "%4d cm" % o["distance"],
               cfg.TARGET_DISTANCE_CM, o["strength"]),
            "  (closer than want -> forward NEGATIVE = back off)",
            "lock    %s   range gate %s"
            % ("YES" if o["locked"] else "no ",
               "OPEN" if o["gate_open"] else "shut"),
            "yaw     %+6.1f deg/s  %s" % (o["yaw_rate"], o["yaw_hint"]),
            "forward %+6.2f m/s   [%s]" % (o["forward"], o["reason"]),
        ]
        for i, text in enumerate(lines):
            y = 18 + i * 17
            cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        (0, 0, 0), 3)
            cv2.putText(img, text, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                        WHITE, 1)


def cmd_track(args):
    """Live tracking preview with the computed commands."""
    import camera

    # Runtime tuning. camera.py and tracker.py read these from cfg on every
    # call, so mutating them here takes effect immediately and nothing has
    # to be edited on disk to try a value.
    if args.conf is not None:
        cfg.CONF_THRESHOLD = args.conf
    if args.min_area is not None:
        cfg.MIN_BOX_AREA_PX = args.min_area
    if args.hgate_frac is not None:
        cfg.LIDAR_HGATE_FRAC = args.hgate_frac
    print("filter: conf>=%.2f  min_area=%d  hgate_frac=%.2f"
          % (cfg.CONF_THRESHOLD, cfg.MIN_BOX_AREA_PX,
             cfg.LIDAR_HGATE_FRAC))

    if args.send:
        print("\n*** --send is ENABLED: real setpoints will be sent. ***")
        print("*** PROPELLERS MUST BE OFF. ***")
        if input("Type SEND to continue: ").strip() != "SEND":
            print("aborted")
            return 1
        connect_or_die()
    else:
        try:
            drone.connect(timeout=5.0)
        except SystemExit:
            pass
        if not drone.is_connected():
            print("No Pixhawk link -- continuing anyway, commands will be "
                  "computed but not sent.")

    try:
        lidar.init()
    except Exception as exc:
        print("LiDAR unavailable (%s); range will read '----'." % exc)

    cfg.DEBUG_PREVIEW = not args.no_preview
    cfg.DEBUG_DRAW = False          # this tool draws its own richer overlay
    width, height = camera.intitalise()
    tracker.configure(width, height)

    if not args.no_preview:
        camera.picam2.pre_callback = _chained_callback

    print("\nWalk in and out of frame. Ctrl-C to stop.")
    print("Overlay: white = image centre, grey = yaw deadband,")
    print("         yellow = forward-alignment limit, cyan = LiDAR boresight.")
    print("Target box is GREEN when the range gate is open, RED when shut.\n")

    controller.reset()
    last_dist = None
    last_dist_t = None
    frames = 0
    fps_t = time.monotonic()
    fps = 0.0

    try:
        while True:
            now = time.monotonic()
            if drone.is_connected():
                drone.poll()

            ts, persons = camera.get_latest()
            fresh = persons is not None and (now - ts) <= cfg.VISION_MAX_AGE_S
            target = tracker.select(persons) if fresh else None

            error_x = error_y = 0.0
            locked = False
            gate_open = False

            if target is not None:
                dist_for_center = (last_dist
                                   if last_dist_t is not None
                                   and now - last_dist_t <= cfg.LIDAR_MAX_AGE_S
                                   else None)
                lock_center = tracker.get_lock_center(dist_for_center)
                locked, error_x, error_y = tracker.is_locked(target,
                                                             lock_center)
                # is_locked() already covers both axes, sized to the target box.
                gate_open = locked
                if gate_open:
                    reading = lidar.read_data()
                    if reading is not None:
                        last_dist = reading
                        last_dist_t = now
            else:
                lidar.read_data()      # keep the serial buffer drained

            distance = (last_dist
                        if last_dist_t is not None
                        and now - last_dist_t <= cfg.LIDAR_MAX_AGE_S
                        else None)

            telemetry = controller.update(
                error_x if target is not None else 0.0,
                distance,
                yaw_only=args.yaw_only,
                send=bool(args.send and drone.is_connected()),
                now=now,
                target_width_px=(target.width if target is not None
                                 else None),
            )

            frames += 1
            if now - fps_t >= 0.5:
                fps = frames / (now - fps_t)
                frames = 0
                fps_t = now

            if target is not None:
                status = "LOCKED" if gate_open else "tracked"
            elif persons:
                # Detector saw people; association rejected them all.
                status = "ASSOC-FAIL"
            elif fresh:
                status = "NO-DETECTION"
            else:
                status = "VISION-STALE"

            _overlay.update({
                "target": target,
                "persons": persons or [],
                "status": status,
                "misses": tracker.misses(),
                "deadband_px": telemetry.get("deadband_px",
                                             cfg.YAW_DEADBAND_PX),
                "hgate_px": (tracker.hgate_px(target.width)
                             if target is not None
                             else cfg.LIDAR_HGATE_MIN_PX),
                "vgate_px": (tracker.vgate_px(target.height)
                             if target is not None
                             else cfg.LIDAR_VGATE_MIN_PX),
                "error_x": error_x,
                "error_y": error_y,
                "distance": distance,
                "strength": lidar.last_strength(),
                "locked": locked,
                "gate_open": gate_open,
                "yaw_rate": telemetry["yaw_rate"],
                "yaw_hint": yaw_hint(telemetry["yaw_rate"]),
                "forward": telemetry["forward"],
                "reason": telemetry["gate"],
                "fps": fps,
                "sending": bool(args.send),
            })

            if args.no_preview:
                print("\r%-12s n=%-2d miss=%-2d err_x %+7.1f err_y %+7.1f  "
                      "lidar %s gate %-4s yaw %+6.1f fwd %+5.2f  %s"
                      % (status, len(persons or []), tracker.misses(),
                         error_x, error_y,
                         "----" if distance is None else "%4d" % distance,
                         "OPEN" if gate_open else "shut",
                         telemetry["yaw_rate"], telemetry["forward"],
                         yaw_hint(telemetry["yaw_rate"])),
                      end="", flush=True)

            time.sleep(max(0.0, cfg.TICK_PERIOD_S - (time.monotonic() - now)))
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if args.send and drone.is_connected():
            drone.stop()
        lidar.close()
        camera.stop()
        drone.disconnect_drone()
    return 0


def _chained_callback(request):
    """Publish detections via camera.py, then draw this tool's overlay."""
    import camera
    camera._on_frame(request)
    try:
        _draw(request)
    except Exception:
        _log.exception("overlay failed")


# ----------------------------------------------------------------- motors ----

MOTOR_TEST_THROTTLE_PERCENT = 0

# ArduPilot Quad X (FRAME_CLASS=1, FRAME_TYPE=1) motor layout and rotation.
# A propeller spinning CCW applies a CW reaction torque to the frame, so to yaw
# the airframe clockwise (nose right) the CCW motors speed up.
QUAD_X = {
    1: ("front-right", "CCW"),
    2: ("rear-left", "CCW"),
    3: ("front-left", "CW"),
    4: ("rear-right", "CW"),
}
YAW_RIGHT_MOTORS = (1, 2)   # CCW props -> CW (nose-right) reaction torque
YAW_LEFT_MOTORS = (3, 4)    # CW props  -> CCW (nose-left) reaction torque


def yaw_hint(yaw_rate):
    """Describe what a commanded yaw rate should do physically."""
    if abs(yaw_rate) < 0.05:
        return "hold"
    if yaw_rate > 0:
        return "nose RIGHT (CW) -> M%d,M%d speed up" % YAW_RIGHT_MOTORS
    return "nose LEFT (CCW) -> M%d,M%d speed up" % YAW_LEFT_MOTORS


def print_motor_map():
    hr("QUAD X MOTOR MAP (FRAME_CLASS=1, FRAME_TYPE=1)")
    for n in sorted(QUAD_X):
        position, spin = QUAD_X[n]
        print("  M%d  %-12s prop spins %s" % (n, position, spin))
    print("\n  A CCW prop pushes the frame CW, so:")
    print("    yaw RIGHT (positive yaw_rate) -> M%d and M%d speed up"
          % YAW_RIGHT_MOTORS)
    print("    yaw LEFT  (negative yaw_rate) -> M%d and M%d speed up"
          % YAW_LEFT_MOTORS)


def cmd_motors(args):
    """Spin each motor briefly using ArduPilot's motor test. PROPS OFF."""
    hr("MOTOR TEST -- PROPELLERS MUST BE REMOVED")
    print("This spins motors one at a time at %d%% for %.1f s each."
          % (args.throttle, args.duration))
    print("It uses MAV_CMD_DO_MOTOR_TEST while DISARMED, the same mechanism")
    print("as Mission Planner's motor test screen.\n")
    print("Before continuing, confirm:")
    print("  - every propeller is off the aircraft")
    print("  - the aircraft is secured and cannot move")
    print("  - nothing is near the motors")
    print("  - the battery is connected (ESCs need power)\n")

    if input('Type REMOVED to confirm the props are off: ').strip() != "REMOVED":
        print("aborted")
        return 1

    connect_or_die()
    drone.poll()

    if drone.is_armed():
        print("\nREFUSING: the vehicle is ARMED. Disarm before a motor test.")
        return 1

    if args.yaw_right:
        motors = list(YAW_RIGHT_MOTORS)
        print("\nSpinning the YAW-RIGHT pair: these are the motors that must")
        print("speed up when the tracker commands a positive yaw rate.")
    elif args.yaw_left:
        motors = list(YAW_LEFT_MOTORS)
        print("\nSpinning the YAW-LEFT pair.")
    elif args.motor:
        motors = [args.motor]
    else:
        motors = list(range(1, args.count + 1))

    print_motor_map()
    print("\nArduPilot numbers motors by its own frame layout, not by your")
    print("wiring order. Check each result against the motor-order diagram")
    print("for FRAME_CLASS/FRAME_TYPE before concluding anything is miswired.")
    print("\nWatch for: does it spin, and which way does it turn?\n")

    for motor in motors:
        input("Press Enter to spin motor %d ... " % motor)
        drone.master.mav.command_long_send(
            drone.master.target_system,
            drone.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
            0,
            float(motor),                       # param1: motor number, 1-based
            float(MOTOR_TEST_THROTTLE_PERCENT),  # param2: throttle type
            float(args.throttle),               # param3: throttle value
            float(args.duration),               # param4: timeout, seconds
            0.0,                                # param5: motor count
            0.0,                                # param6: test order
            0.0,
        )

        ack = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            msg = drone.master.recv_match(type="COMMAND_ACK", blocking=True,
                                          timeout=0.3)
            if msg is not None and msg.command == \
                    mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST:
                ack = msg
                break

        if ack is None:
            print("  motor %d: no COMMAND_ACK received" % motor)
        elif ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("  motor %d: ACCEPTED" % motor)
        else:
            print("  motor %d: REJECTED (result=%d)" % (motor, ack.result))
            print("    Common causes: safety switch still engaged, ESCs "
                  "unpowered,")
            print("    FRAME_CLASS/FRAME_TYPE unset, or throttle below "
                  "MOT_SPIN_ARM.")

        time.sleep(args.duration + 0.3)

    hr("DONE")
    print("If a motor spun the wrong way, reverse it in BLHeliSuite or swap")
    print("any two of its three phase wires. If a motor did not spin at all,")
    print("raise --throttle a little (MOT_SPIN_ARM / MOT_SPIN_MIN set the")
    print("floor) before assuming a wiring fault.")
    drone.disconnect_drone()
    return 0


# ------------------------------------------------------------------- main ----

def main(argv=None):
    parser = argparse.ArgumentParser(
        description="BILBO bench diagnostics (ground only, props off)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("-v", "--verbose", action="store_true")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("link", help="MAVLink connection and stream rates")
    p.add_argument("--seconds", type=float, default=5.0,
                   help="listen window for measuring message rates")
    p.set_defaults(func=cmd_link)

    p = sub.add_parser("switch", help="live ch%d readout" % cfg.CH_ENABLE)
    p.set_defaults(func=cmd_switch)

    p = sub.add_parser("sensors", help="LiDAR + camera health, text only")
    p.set_defaults(func=cmd_sensors)

    p = sub.add_parser("leds", help="walk through every LED state")
    p.add_argument("--dwell", type=float, default=2.0,
                   help="seconds to hold each state (default 2)")
    p.set_defaults(func=cmd_leds)

    p = sub.add_parser("track", help="live tracking preview and commands")
    p.add_argument("--send", action="store_true",
                   help="actually send setpoints (props off, asks to confirm)")
    p.add_argument("--yaw-only", action="store_true",
                   help="force forward velocity to zero")
    p.add_argument("--no-preview", action="store_true",
                   help="text only, for use over SSH with no display")
    p.add_argument("--conf", type=float,
                   help="override CONF_THRESHOLD (lower = more detections)")
    p.add_argument("--min-area", type=int,
                   help="override MIN_BOX_AREA_PX")
    p.add_argument("--hgate-frac", type=float,
                   help="override LIDAR_HGATE_FRAC (range gate width)")
    p.set_defaults(func=cmd_track)

    p = sub.add_parser("motors", help="spin motors briefly, PROPS OFF")
    p.add_argument("--throttle", type=int, default=10,
                   help="percent (default 10)")
    p.add_argument("--duration", type=float, default=2.0,
                   help="seconds per motor (default 2)")
    p.add_argument("--count", type=int, default=4,
                   help="number of motors (default 4)")
    p.add_argument("--motor", type=int,
                   help="test only this motor number")
    p.add_argument("--yaw-right", action="store_true",
                   help="spin only the pair that yaws the nose right")
    p.add_argument("--yaw-left", action="store_true",
                   help="spin only the pair that yaws the nose left")
    p.set_defaults(func=cmd_motors)

    args = parser.parse_args(argv)
    setup_logging(args.verbose)
    try:
        return args.func(args)
    except KeyboardInterrupt:
        print("\ninterrupted")
        return 130


if __name__ == "__main__":
    sys.exit(main())
