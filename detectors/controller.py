DEADBAND = 50
MIN_DISTANCE = 100  # centimeters
MAX_DISTANCE = 110 
import drone

def update(error_x, distance):

    if abs(error_x) > DEADBAND:

        if error_x > 0:
            drone.yaw_right()
        else:
            drone.yaw_left()

        return

    # Yaw is aligned from here on

    if distance is None:
        drone.hover()
        return

    if distance < MIN_DISTANCE:
        drone.backward()

    elif distance > MAX_DISTANCE:
        drone.forward()

    else:
        drone.hover()