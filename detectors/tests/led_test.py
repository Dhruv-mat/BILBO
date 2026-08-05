from pi5neo import Pi5Neo
import time

neo = Pi5Neo("/dev/spidev0.0", num_leds=25, brightness=0.3, spi_speed_khz=800)

while True:

    # One quick flash
    neo.fill_strip(255, 255, 255)
    neo.update_strip()
    time.sleep(0.05)

    neo.fill_strip(180, 0, 0)
    neo.update_strip()
    time.sleep(0.25)

    # Two quick flashes
    for _ in range(2):
        neo.fill_strip(255, 255, 255)
        neo.update_strip()
        time.sleep(0.03)

        neo.fill_strip(180, 0, 0)
        neo.update_strip()
        time.sleep(0.08)

    time.sleep(0.2)

    # Three rapid flashes
    for _ in range(3):
        neo.fill_strip(255, 255, 255)
        neo.update_strip()
        time.sleep(0.02)

        neo.fill_strip(255, 0, 0)
        neo.update_strip()
        time.sleep(0.05)

    # Hold red
    time.sleep(0.6)