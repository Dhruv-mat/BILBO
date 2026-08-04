from pi5neo import Pi5Neo
import time

neo = Pi5Neo("/dev/spidev0.0", num_leds=25, spi_speed_khz=800)

# Clear strip
neo.clear_strip()
neo.update_strip()

time.sleep(1)

# Red
neo.fill_strip(255, 0, 0)
neo.update_strip()
time.sleep(2)

# Green
neo.fill_strip(0, 255, 0)
neo.update_strip()
time.sleep(2)

# Blue
neo.fill_strip(0, 0, 255)
neo.update_strip()
time.sleep(2)

# White
neo.fill_strip(255, 255, 255)
neo.update_strip()
time.sleep(2)

# Off
neo.clear_strip()
neo.update_strip()