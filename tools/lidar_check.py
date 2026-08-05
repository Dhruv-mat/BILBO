"""Props-off validation of the TF Luna read path.

Checks the four things that were broken before: frame synchronisation, checksum
validation, newest-frame-wins, and strength/range gating.

    PROCEDURE
    1.  Run it and hold a flat target at 1 m, 2 m, 4 m, 6 m. Reported distance
        should track a tape measure within a few centimetres.
    2.  Aim at open sky. Expect REJECTED to climb and distance to read "--":
        low signal strength must not produce a number.
    3.  Aim at a dark garment in bright sunlight. This is the dominant outdoor
        failure mode -- confirm it rejects rather than inventing a range.
    4.  RESYNC TEST: unplug the LiDAR's data line for a second and reconnect it
        mid-run. The old driver would desync permanently and return None for the
        rest of the flight. This one must recover within a frame or two.
    5.  Confirm BAD_CHECKSUM stays at or near zero on a good cable. A steadily
        rising count means electrical noise -- check routing away from the
        NeoPixel SPI line and the power wiring.
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                os.pardir, "detectors"))

import config as cfg           # noqa: E402
import lidar                   # noqa: E402


def main():
    print(__doc__)
    print("device %s @ %d, accept %d-%d cm, min strength %d"
          % (cfg.LIDAR_DEVICE, cfg.LIDAR_BAUD, cfg.LIDAR_MIN_CM,
             cfg.LIDAR_MAX_CM, cfg.LIDAR_MIN_STRENGTH))
    print()

    try:
        lidar.init()
    except Exception as exc:
        print("FAIL: could not open %s: %s" % (cfg.LIDAR_DEVICE, exc))
        print("  If this says 'device or resource busy', something else has "
              "the port open -- check it is not the MAVLink device.")
        return 1

    last_valid = None
    last_valid_time = None
    gaps = 0

    try:
        while True:
            now = time.monotonic()
            distance = lidar.read_data()
            health = lidar.health()

            if distance is not None:
                if (last_valid_time is not None
                        and now - last_valid_time > 0.5):
                    gaps += 1
                    print("  (recovered after %.2fs gap -- resync works)"
                          % (now - last_valid_time))
                last_valid = distance
                last_valid_time = now

            age = "--" if last_valid_time is None else "%.2fs" % (
                now - last_valid_time)
            shown = "--" if distance is None else "%4d cm" % distance
            stale = (last_valid_time is not None
                     and now - last_valid_time > cfg.LIDAR_MAX_AGE_S)

            print("now %s | last_valid %s (age %s)%s | "
                  "valid %d  bad_checksum %d  rejected %d  gaps %d"
                  % (shown,
                     "--" if last_valid is None else "%4d cm" % last_valid,
                     age,
                     "  STALE->would command zero forward" if stale else "",
                     health["valid"], health["bad_checksum"],
                     health["rejected"], gaps))

            time.sleep(1.0 / 15.0)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        lidar.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
