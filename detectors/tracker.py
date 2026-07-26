

def is_locked(person,cam_x_cent , cam_y_cent):
    print("is_locked working")
    left = person.x
    right = person.x + person.width

    top = person.y
    bottom = person.y + person.height
        
    if (
        left <= cam_x_cent//2 <= right
            and
        top <= cam_y_cent//2 <= bottom
    ):
        print("centered")
        return True
    else:
        print(f"Camera center : ({cam_x_cent}, {cam_y_cent})")

        print(f"Person left   : {left}")
        print(f"Person right  : {right}")
        print(f"Person top    : {top}")
        print(f"Person bottom : {bottom}")

        print(f"Person center : ({person.center_x}, {person.center_y})")
        return False

def select(persons):
    print("select working")
    if not persons:
        return None

    return max(persons, key=lambda p: p.area)