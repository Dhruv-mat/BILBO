DEADBAND = 5
import drone

def update(error_x, last_good_distance):
    print(f"controller: {error_x}")
    if abs(error_x) <= DEADBAND:
        # drone.hover()
        print ("hover")

    elif error_x > DEADBAND:
        # drone.yaw_right()
        print("right")

    else:
        # drone.yaw_left()
        print("left")