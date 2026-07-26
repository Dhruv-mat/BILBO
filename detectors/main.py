import lidar
import camera
import tracker
print("hi")
cam_x_cent , cam_y_cent = camera.intitalise()

while True:
    persons = camera.get_people()

    target = tracker.select(persons)

    if target is None:
        continue

    if tracker.is_locked(target,cam_x_cent , cam_y_cent):
        distance = lidar.read_data()
