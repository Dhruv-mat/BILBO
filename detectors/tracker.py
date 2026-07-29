import math

BASELINE = 0.05
CAMERAFOV = 78 #this is in degress fyi
IMAGE_WIDTH = 640
PPD = IMAGE_WIDTH/CAMERAFOV
LOCK_THRESHOLD = 15

def get_lock_center(distance):
    angle = math.degrees(math.atan(BASELINE / distance))

    pixel_offset = angle * PPD
    if distance is None or distance <= 0:

        return IMAGE_WIDTH // 2 + pixel_offset


# def is_locked(person,lock_center):
#     print("is_locked working")
#     left = person.x
#     right = person.x + person.width

#     top = person.y
#     bottom = person.y + person.height
        
#     if (
#         left <= lock_center <= right
#         ):
#         return True
#     else:
#         # print(f"Camera center : ({cam_x_cent}, {cam_y_cent})")

#         # print(f"Person left   : {left}")
#         # print(f"Person right  : {right}")
#         # print(f"Person top    : {top}")
#         # print(f"Person bottom : {bottom}")

#         # print(f"Person center : ({person.center_x}, {person.center_y})")
#         return False

def is_locked(person, camera_center_x):

    if camera_center_x is None:

        return False, 0

    print(person.center_x)
    error_x = person.center_x - camera_center_x

    print(

    f"person_center={person.center_x}, "

    f"camera_center={camera_center_x}, "

    f"error={error_x}"

)
    


    locked = (

        abs(error_x) < LOCK_THRESHOLD


    )

    return locked, error_x

def select(persons):
    if not persons:
        return None
    else:
        return persons

    return max(persons, key=lambda p: p.area)