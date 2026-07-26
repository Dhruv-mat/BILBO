

def is_locked(person,cam_x_cent , cam_y_cent):

    left = person.x
    right = person.x + person.width

    top = person.y
    bottom = person.y + person.height
        
    if (
        left <= cam_x_cent <= right
            and
        top <= cam_y_cent <= bottom
    ):
        return True
    else:
        return False

def select(persons):

    if not persons:
        return None

    return max(persons, key=lambda p: p.area)