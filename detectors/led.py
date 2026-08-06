
import inspect
import logging
import time

import config as cfg
from state import DroneState

_log = logging.getLogger(__name__)

try:
    from pi5neo import Pi5Neo
except Exception:  
    Pi5Neo = None

neo = None
_available = False
_last_written = None

_hw_brightness = False     
_fill_takes_tuple = False

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

_STATE_APPEARANCE = {
    DroneState.IDLE: ("solid", "white"),
    DroneState.READY: ("solid", "blue"),
    DroneState.TRACKING: ("solid", "green"),
    DroneState.SEARCHING: ("blink", "yellow"),
    DroneState.RTL: ("solid", "purple"),
    DroneState.LANDING: ("blink", "purple"),
    DroneState.EMERGENCY: ("blink", "red"),
}


def _accepted_params(func):
    try:
        return set(inspect.signature(func).parameters)
    except (TypeError, ValueError):
        return set()


def init():
    """Open the strip. Never raises -- LEDs are not flight critical."""
    global neo, _available, _hw_brightness, _fill_takes_tuple

    if Pi5Neo is None:
        _log.warning("pi5neo unavailable; LED output disabled")
        return False

    accepted = _accepted_params(Pi5Neo.__init__)
    kwargs = {}
    if "num_leds" in accepted:
        kwargs["num_leds"] = cfg.LED_COUNT
    if "spi_speed_khz" in accepted:
        kwargs["spi_speed_khz"] = cfg.LED_SPI_KHZ
    if "brightness" in accepted:
        kwargs["brightness"] = cfg.LED_BRIGHTNESS
        _hw_brightness = True
    else:
        
        _hw_brightness = False

    try:
        neo = Pi5Neo(cfg.LED_DEVICE, **kwargs)
    except Exception:
        _log.exception("LED init failed; continuing without LEDs")
        neo = None
        _available = False
        return False

    
    fill_params = _accepted_params(getattr(neo, "fill_strip", None))
    _fill_takes_tuple = len(fill_params) < 3

    _available = True
    _log.info("LED strip ready on %s (%d leds, brightness in %s, "
              "fill_strip takes %s)",
              cfg.LED_DEVICE, cfg.LED_COUNT,
              "hardware" if _hw_brightness else "software",
              "a tuple" if _fill_takes_tuple else "r,g,b")
    blank()
    return True

def is_available():
    return _available

def _scaled(rgb):
    if _hw_brightness:
        return rgb
    b = cfg.LED_BRIGHTNESS
    return (int(rgb[0] * b), int(rgb[1] * b), int(rgb[2] * b))


def _write(rgb):
    global _last_written

    if not _available or rgb == _last_written:
        return
    out = _scaled(rgb)
    try:
        if _fill_takes_tuple:
            neo.fill_strip(out)
        else:
            neo.fill_strip(*out)
        neo.update_strip()
        _last_written = rgb
    except Exception:
        _log.exception("LED CONNECTION failed; disabling LED output")
        _disable()

def _disable():
    global _available
    _available = False

def blank():
    _write(colours["off"])

def selftest():
    if not _available:
        return
    for name in ("red", "green", "blue"):
        _write(colours[name])
        time.sleep(0.15)
    blank()

def note_tick():
    global _ticks, _last_tick_time
    _ticks += 1
    _last_tick_time = time.monotonic()

def _loop_alive(now):
    return _last_tick_time != 0.0 and (now - _last_tick_time) < LIVENESS_MAX_AGE_S

def _liveness_overrides(now):
    if not _loop_alive(now):
        return False
    phase = now % LIVENESS_PERIOD_S
    first_end = LIVENESS_FLASH_S
    second_start = LIVENESS_FLASH_S + LIVENESS_GAP_S
    second_end = second_start + LIVENESS_FLASH_S
    return phase < first_end or (second_start <= phase < second_end)

def led_status(effect, color):
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
    if not _available:
        return
    now = time.monotonic()
    if _liveness_overrides(now):
        _write(colours["white"])
        return
    if fault is not None:
        led_status("blink", "orange")
        return
    effect, color = _STATE_APPEARANCE.get(state, ("blink", "orange"))
    led_status(effect, color)

def shutdown():
    blank()
