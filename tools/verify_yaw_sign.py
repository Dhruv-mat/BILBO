"""Props-off confirmation of the yaw sign convention.

This is the single most important bench test before the first autonomous
flight. The pre-review code had an INVERTED yaw sign, which makes the control
loop positive feedback: the drone spins away from the target until it leaves
frame, then enters SEARCHING and keeps spinning.

Nothing here talks to the Pixhawk. No setpoint is sent. Safe to run with the
battery disconnected and the camera on USB power.

    PROCEDURE
    1.  Run this with the drone on a bench, nose pointing at the far wall.
    2.  Stand to the drone's PHYSICAL RIGHT, inside the camera's view.
    3.  Confirm the tool reports:  image RIGHT  and  nose RIGHT  ->  CORRECT
    4.  Repeat standing to the drone's PHYSICAL LEFT.
        Confirm:  image LEFT  and  nose LEFT  ->  CORRECT

    If the image side does not match the side you are physically standing on,
    the camera mounting/transform assumption in config.CAMERA_IS_INVERTED is
    wrong -- fix that first, the yaw sign is downstream of it.

    If the image side is right but the nose direction is opposite, flip
    config.YAW_PID_OUTPUT_SIGN and re-run.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "detectors"))

import config as cfg           # noqa: E402
import camera                  # noqa: E402
import controller              # noqa: E402
import tracker                 # noqa: E402


def side(value, positive, negative, dead="centre"):
    if abs(value) < 1e-6:
        return dead
    return positive if value > 0 else negative


def main():
    print(__doc__)
    print("config: CAMERA_IS_INVERTED=%s  YAW_PID_OUTPUT_SIGN=%+.1f"
          % (cfg.CAMERA_IS_INVERTED, cfg.YAW_PID_OUTPUT_SIGN))
    print("        YAW_KP=%.3f  MAX_YAW_RATE=%.1f deg/s  DEADBAND=%.0f px"
          % (cfg.YAW_KP, cfg.MAX_YAW_RATE_DEG_S, cfg.YAW_DEADBAND_PX))
    print()

    width, height = camera.intitalise()
    tracker.configure(width, height)
    print("camera %dx%d, waiting for inference..." % (width, height))

    deadline = time.monotonic() + 10.0
    while not camera.inference_alive() and time.monotonic() < deadline:
        time.sleep(0.1)
    if not camera.inference_alive():
        print("FAIL: no inference output from the IMX500")
        return 1

    print("ready -- stand to one side of the drone\n")

    try:
        while True:
            now = time.monotonic()
            ts, persons = camera.get_latest()

            if persons is None:
                print("vision UNHEALTHY (no inference output this frame)")
                time.sleep(0.2)
                continue

            target = tracker.select(persons)
            if target is None:
                print("no target (%d detections)" % len(persons))
                time.sleep(0.2)
                continue

            # Distance is irrelevant to the sign test; pass None so forward
            # velocity stays gated off.
            error_x = target.center_x - (cfg.IMAGE_WIDTH_PX / 2.0)
            telemetry = controller.update(error_x, None, send=False)
            yaw = telemetry["yaw_rate"]

            image_side = side(error_x, "RIGHT", "LEFT")
            nose_side = side(yaw, "RIGHT", "LEFT", dead="HOLD")

            if abs(error_x) < cfg.YAW_DEADBAND_PX:
                verdict = "in deadband -- move further off centre"
            elif (error_x > 0) == (yaw > 0):
                verdict = "CORRECT (turning toward the target)"
            else:
                verdict = ("*** INVERTED *** turning AWAY -- flip "
                           "config.YAW_PID_OUTPUT_SIGN")

            print("age %4.0f ms | error_x %+7.1f px (image %-6s) | "
                  "yaw %+6.1f deg/s (nose %-5s) | %s"
                  % ((now - ts) * 1000.0, error_x, image_side,
                     yaw, nose_side, verdict))

            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        camera.stop()

    return 0


if __name__ == "__main__":
    sys.exit(main())
