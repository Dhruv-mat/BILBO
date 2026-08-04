from rpi_ws281x import PixelStrip, Color
import time

# LED configuration
LED_COUNT = 25          # Total number of LEDs
LED_PIN = 18            # GPIO18
LED_FREQ_HZ = 800000
LED_DMA = 10
LED_BRIGHTNESS = 150
LED_INVERT = False
LED_CHANNEL = 0

strip = PixelStrip(
    LED_COUNT,
    LED_PIN,
    LED_FREQ_HZ,
    LED_DMA,
    LED_INVERT,
    LED_BRIGHTNESS,
    LED_CHANNEL
)

strip.begin()


def set_all(r, g, b):
    for i in range(LED_COUNT):
        strip.setPixelColor(i, Color(r, g, b))
    strip.show()


# -------- Startup Animation --------

set_all(0, 0, 0)
time.sleep(0.8)

# Power flows through
for i in range(LED_COUNT):
    strip.setPixelColor(i, Color(120, 0, 0))
    strip.show()
    time.sleep(0.05)

time.sleep(0.4)

# White flash
set_all(255, 255, 255)
time.sleep(0.05)

set_all(0, 0, 0)
time.sleep(0.12)

# Deep red
set_all(180, 0, 0)
time.sleep(0.5)

# Double flash
for _ in range(2):
    set_all(255, 255, 255)
    time.sleep(0.03)
    set_all(180, 0, 0)
    time.sleep(0.10)

# -------- Idle "heartbeat" --------

while True:
    for b in range(60, 151, 2):
        strip.setBrightness(b)
        strip.show()
        time.sleep(0.02)

    for b in range(150, 59, -2):
        strip.setBrightness(b)
        strip.show()
        time.sleep(0.02)