from enum import Enum

class DroneState(Enum):
    IDLE = 0
    READY = 1
    TRACKING = 2
    SEARCHING = 3
    RTL = 4
    LANDING = 5
    EMERGENCY = 6
