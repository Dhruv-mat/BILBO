import lidar
import camera
import tracker

cam_x_cent , cam_y_cent = camera.initialize()


lidar.initialize()



while True:
    persons = camera.get_people()

    target = tracker.select(persons)

    if target is None:
        continue

    if tracker.is_locked(target,cam_x_cent , cam_y_cent):
        distance = lidar.read_data()
