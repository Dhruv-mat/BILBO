from pi5neo import Pi5Neo
import time

neo = Pi5Neo("/dev/spidev0.0", num_leds=25, brightness=0.3, spi_speed_khz=800)

colours = {

    'yellow': (255,255,0),
    'red': (255,0,0),
    'green': (0,255,0),
    'blue': (0,0,255),
    'purple': (128,0,128),
    'white': (255,255,255),
    'black': (0,0,0),
    'orange':(255,165,0)
}

def led_statur(state,color):
    if state == "blink":
        

