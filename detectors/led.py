"""Pi5Neo status strip.

Fixes four defects, two of which were unconditional crashes:

  1. The function signature was led_status(state, color) while every call site
     passed effect=..., so every call raised TypeError.
  2. fill_strip was called with a single tuple, but pi5neo's signature is
     fill_strip(r, g, b) -- as tests/led_test.py correctly uses. So every LED
     write raised TypeError for a second, independent reason.
  3. The "solid" branch toggled led_on on every call, so solid green rendered
     as a strobe at loop rate.
  4. update_strip() was called every iteration even when nothing had changed --
     a blocking SPI write at loop rate for no reason.

Animations are stateless functions of time.monotonic(), so there is no shared
blink phase to get out of step and no sleeps anywhere in the flight path.

LED failure must never stop the aircraft, so every entry point degrades to a
no-op if the strip is unavailable.
"""

import logging
import time

import config as cfg
from state import DroneState

_log = logging.getLogger(__name__)

try:
    from pi5neo import Pi5Neo
except Exception:  # pragma: no cover - bench machines have no pi5neo
    Pi5Neo = None

neo = None
_available = False
_last_written = None

# Liveness proof. Incremented only by the control loop, so if the loop stalls
# the flashes stop and the pilot gets an immediate visual cue for exactly the
# failure modes that are otherwise silent.
_ticks = 0
_last_tick_time = 0.0

BLINK_PERIOD_S = 0.6
LIVENESS_PERIOD_S = 3.0
LIVENESS_FLASH_S = 0.04
LIVENESS_GAP_S = 0.09
LIVENESS_MAX_AGE_S = 0.5

colours = {
    "yellow": (255, 255, 0),
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "purple": (128, 0, 128),
    "white": (255, 255, 255),
    "black": (0, 0, 0),
    "orange": (255, 165, 0),
    "off": (0, 0, 0),
}

# Single source of truth for state -> appearance. The original scattered LED
# calls across state branches, so entering TRACKING briefly showed blue (the
# READY colour) and READY set no LED at all, leaving whatever the previous state
# had written on the strip.
_STATE_APPEARANCE = {
    DroneState.IDLE: ("solid", "white"),
    DroneState.READY: ("solid", "blue"),
    DroneState.TRACKING: ("solid", "green"),
    DroneState.SEARCHING: ("blink", "yellow"),
    DroneState.RTL: ("solid", "purple"),
    DroneState.LANDING: ("blink", "purple"),
    DroneState.EMERGENCY: ("blink", "red"),
}


def init():
    """Open the strip. Never raises -- LEDs are not flight critical."""
    global neo, _available

    if Pi5Neo is None:
        _log.warning("pi5neo unavailable; LED output disabled")
        return False

    try:
        neo = Pi5Neo(cfg.LED_DEVICE, num_leds=cfg.LED_COUNT,
                     brightness=cfg.LED_BRIGHTNESS,
                     spi_speed_khz=cfg.LED_SPI_KHZ)
        _available = True
        blank()
        return True
    except Exception:
        _log.exception("LED init failed; continuing without LEDs")
        neo = None
        _available = False
        return False


def is_available():
    return _available


def _write(rgb):
    """Push to the strip only when the pixel data actually changed."""
    global _last_written

    if not _available or rgb == _last_written:
        return
    try:
        neo.fill_strip(*rgb)
        neo.update_strip()
        _last_written = rgb
    except Exception:
        _log.exception("LED write failed; disabling LED output")
        _disable()


def _disable():
    global _available
    _available = False


def blank():
    _write(colours["off"])


def selftest():
    """Prove the strip works before the pilot relies on it as a READY cue.

    Blocking, but this runs during preflight on the ground only -- never in the
    flight loop.
    """
    if not _available:
        return
    for name in ("red", "green", "blue"):
        _write(colours[name])
        time.sleep(0.15)
    blank()


def note_tick():
    """Called at the end of every successful control tick, nowhere else."""
    global _ticks, _last_tick_time
    _ticks += 1
    _last_tick_time = time.monotonic()


def _loop_alive(now):
    return _last_tick_time != 0.0 and (now - _last_tick_time) < LIVENESS_MAX_AGE_S


def _liveness_overrides(now):
    """True while a liveness flash should be showing."""
    if not _loop_alive(now):
        return False
    phase = now % LIVENESS_PERIOD_S
    first_end = LIVENESS_FLASH_S
    second_start = LIVENESS_FLASH_S + LIVENESS_GAP_S
    second_end = second_start + LIVENESS_FLASH_S
    return phase < first_end or (second_start <= phase < second_end)


def led_status(effect, color):
    """Set the strip. `effect` is "solid" or "blink".

    Parameter is named `effect` to match the existing call sites that passed
    effect=... against the old `state` parameter name.
    """
    if not _available:
        return

    rgb = colours.get(color)
    if rgb is None:
        _log.warning("unknown LED colour %r", color)
        return

    if effect == "blink":
        now = time.monotonic()
        on = int(now / BLINK_PERIOD_S) % 2 == 0
        _write(rgb if on else colours["off"])
    elif effect == "solid":
        _write(rgb)
    else:
        _log.warning("unknown LED effect %r", effect)


def render(state, fault=None):
    """Drive the strip purely from the current state.

    Called once at the end of every loop iteration. Because appearance is a
    function of state alone, the mapping cannot drift from the documented one.
    """
    if not _available:
        return

    now = time.monotonic()

    if _liveness_overrides(now):
        _write(colours["white"])
        return

    if fault is not None:
        # Any latched fault outranks the state colour.
        led_status("blink", "orange")
        return

    effect, color = _STATE_APPEARANCE.get(state, ("blink", "orange"))
    led_status(effect, color)


def shutdown():
    blank()
