from pi5neo import Pi5Neo
import time

# 25 LEDs on SPI (GPIO10 / MOSI)
leds = Pi5Neo("/dev/spidev0.0", 25)

# 25% brightness
leds.brightness = 64

# Turn everything off
leds.fill((0, 0, 0))
time.sleep(1)

# Red
leds.fill((255, 0, 0))
time.sleep(2)

# Green
leds.fill((0, 255, 0))
time.sleep(2)

# Blue
leds.fill((0, 0, 255))
time.sleep(2)

# White
leds.fill((255, 255, 255))
time.sleep(2)

# Off
leds.fill((0, 0, 0))