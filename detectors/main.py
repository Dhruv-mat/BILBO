import lidar
import camera
import tracker
import time
print("hi")
cam_x_cent , cam_y_cent = camera.intitalise()

while True:
    t0 = time.perf_counter()

    persons = camera.get_people()
    t1 = time.perf_counter()
    target = tracker.select(persons)

    if target is None:
        continue

    if tracker.is_locked(target,cam_x_cent , cam_y_cent):
        distance = lidar.read_data()

    t2 = time.perf_counter()

    # print(f"Camera: {(t1-t0)*1000:.1f} ms")

    # print(f"LiDAR : {(t2-t1)*1000:.1f} ms")
