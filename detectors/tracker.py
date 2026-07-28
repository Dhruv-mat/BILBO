import math

BASELINE = 0.05
CAMERAFOV = 78 #this is in degress fyi
IMAGEWIDTH = 640
PPD = IMAGEWIDTH/CAMERAFOV

def get_lock_center(distance):
    angle = math.degrees(math.atan(BASELINE / distance))

    pixel_offset = angle * PPD

    return IMAGE_WIDTH // 2 + pixel_offset


def is_locked(person,lock_center):
    print("is_locked working")
    left = person.x
    right = person.x + person.width

    top = person.y
    bottom = person.y + person.height
        
    if (
        left <= lock_center <= right
        ):
        return True
    else:
        # print(f"Camera center : ({cam_x_cent}, {cam_y_cent})")

        # print(f"Person left   : {left}")
        # print(f"Person right  : {right}")
        # print(f"Person top    : {top}")
        # print(f"Person bottom : {bottom}")

        # print(f"Person center : ({person.center_x}, {person.center_y})")
        return False

def select(persons):
    print("select working")
    if not persons:
        return None

    return max(persons, key=lambda p: p.area)