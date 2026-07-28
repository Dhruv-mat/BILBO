import lidar
import camera
import tracker
import time
print("hi")
cam_x_cent , cam_y_cent = camera.intitalise()
last_good_distance = None
CAMERA_CENTER = cam_x_cent//2
while True:
    t0 = time.perf_counter()

    persons = camera.get_people()
    t1 = time.perf_counter()
    target = tracker.select(persons)

    if target is None:
        continue

    
    if last_good_distance is None:
        lock_center = CAMERA_CENTER

    else:
        # Predict where the LiDAR beam should appear
        lock_center = tracker.get_lock_center(last_good_distance)

    locked = tracker.is_locked(target, lock_center)

    if locked:
        new_distance = lidar.read_data()
        if new_distance is not None:
            last_good_distance = new_distance
    t2 = time.perf_counter()

    # print(f"Camera: {(t1-t0)*1000:.1f} ms")

    # print(f"LiDAR : {(t2-t1)*1000:.1f} ms")
