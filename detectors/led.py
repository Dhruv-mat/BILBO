from pi5neo import Pi5Neo
import time

last_blink = 0
led_on = False

neo = Pi5Neo("/dev/spidev0.0", num_leds=25, brightness=0.3, spi_speed_khz=800)

colours = {

    'yellow': (255,255,0),
    'red': (255,0,0),
    'green': (0,255,0),
    'blue': (0,0,255),
    'purple': (128,0,128),
    'white': (255,255,255),
    'black': (0,0,0),
    'orange':(255,165,0),
    'off': (0,0,0)
}

def led_status(state,color):
    global last_blink, led_on
    if state == "blink":
        if time.monotonic() - last_blink >= 0.3:

            led_on = not led_on
            if led_on:
                neo.fill_strip(colours[color])
            else:
                neo.fill_strip(colours["off"])

            neo.update_strip()
            last_blink = time.monotonic()

    elif state == 'solid':

        led_on = not led_on
        if led_on:
            neo.fill_strip(colours[color])
        else:
            neo.fill_strip(colours["off"])

        neo.update_strip()


