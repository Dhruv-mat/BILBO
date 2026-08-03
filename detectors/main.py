import lidar
import camera
import tracker
import time
import controller
import drone 

print("hi bitches")
cam_x_cent , cam_y_cent = camera.intitalise()
last_good_distance = None
CAMERA_CENTER = cam_x_cent//2

drone.connect()

while True:
    t0 = time.perf_counter()
    persons = camera.get_people()
    t1 = time.perf_counter()
    target = tracker.select(persons)

    if target is None:
        # controller.hover()
        print("No target selected")
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

    # print(f"Camera: {(t1-t0)*1000:.1f} ms")
    # print(f"LiDAR : {(t2-t1)*1000:.1f} ms")
