DEADBAND = 30
import drone

def update(error_x, last_good_distance):
    if abs(error_x) <= DEADBAND:
        # drone.hover()
        print ("hover")
        print(last_good_distance)

    elif error_x > DEADBAND:
        # drone.yaw_right()
        print("right")

    else:
        # drone.yaw_left()
        print("left")