import math

BASELINE = 0.05
CAMERAFOV = 78 #this is in degress fyi
IMAGE_WIDTH = 640
PPD = IMAGE_WIDTH/CAMERAFOV
LOCK_THRESHOLD = 15

def get_lock_center(distance):
    if distance is None or distance <= 0:
        return IMAGE_WIDTH // 2

    angle = math.degrees(math.atan(BASELINE / distance))
    pixel_offset = angle * PPD

    return IMAGE_WIDTH // 2 + pixel_offset

def is_locked(person, camera_center_x):

    if camera_center_x is None:

        return False, 0

    print(person.center_x)
    error_x = person.center_x - camera_center_x


    locked = (  abs(error_x) < LOCK_THRESHOLD )

    return locked, error_x

def select(persons):
    if not persons:
        return None

    return max(persons, key=lambda p: p.area)