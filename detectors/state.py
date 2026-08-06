from enum import Enum

class DroneState(Enum):
    IDLE = 0
    READY = 1
    TRACKING = 2
    # TAKEOFF sits between READY and TRACKING: the Pixhawk owns the climb
    # and the Pi commands nothing until the target altitude is reached.
    TAKEOFF = 7
    SEARCHING = 3
    RTL = 4
    LANDING = 5
    EMERGENCY = 6
