import lidar
import camera
import tracker
import time
import controller
import drone 
import led
from state import DroneState




print("hi bitches")
cam_x_cent , cam_y_cent = camera.intitalise()

drone.connect()
CAMERA_CENTER = cam_x_cent//2
SEARCH_TIMEOUT = 10
SEARCH_YAW_RATE = 8
LOST_FRAME_LIMIT = 6
SWITCH_HIGH = 1800
SWITCH_LOW = 1200

state = DroneState.READY
last_good_distance = None
search_start_time = None
lost_frames = 0

while True:

    ch6 = drone.get_channel(6)
    mode = drone.get_mode()

    if mode != "GUIDED":
        drone.stop()
        state = DroneState.IDLE
        continue

    if state == DroneState.READY:
        if ch6 is not None and ch6 > SWITCH_HIGH:
            print("Tracking enabled")
            led.led_status(effect="solid", color="blue")
            state = DroneState.TRACKING
            continue

    elif state == DroneState.TRACKING:

        persons = camera.get_people()
        target = tracker.select(persons)
        led.led_status(effect="solid", color="green")

        if target is not None:
            lost_frames = 0

        if target is None:
            lost_frames += 1

            if lost_frames >= LOST_FRAME_LIMIT:
                print("starting up searching")
                search_start_time = time.monotonic()
                state = DroneState.SEARCHING
            continue


        if last_good_distance is None:
            lock_center = CAMERA_CENTER
        else:
            lock_center = tracker.get_lock_center(last_good_distance)
        locked, error_x = tracker.is_locked(target, lock_center)

        if abs(error_x) < controller.YAW_DEADBAND:
            
            new_distance = lidar.read_data()
            if new_distance is not None:
                last_good_distance = new_distance

        controller.update(error_x, last_good_distance)

    elif state == DroneState.SEARCHING:

        print("Searching...")
        led.led_status(effect="blink", color="yellow")
        persons = camera.get_people()
        target = tracker.select(persons)

        if target is not None:
            print("Target found!")
            state = DroneState.TRACKING
            continue

        drone.move(
            yaw_rate=SEARCH_YAW_RATE
        )

        if time.monotonic() - search_start_time > SEARCH_TIMEOUT:
            state = DroneState.RTL

    elif state == DroneState.IDLE:
        led.led_status(effect="solid", color="white")
        if mode == "GUIDED":
            print("READY")
            state = DroneState.READY

        continue

    elif state == DroneState.RTL:
        print("RTL")
        led.led_status(effect="solid", color="purple")
        drone.set_mode("RTL")
        continue

    elif state == DroneState.EMERGENCY:
        drone.hover()
        led.led_status(effect="blink", color="red")
        continue
