import argparse
import cv2

def create_person_detector():
    detector = cv2.HOGDescriptor()
    detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    return detector

def find_people(detector, frame):
    boxes, _weights = detector.detectMultiScale(
        frame, winStride = (8, 8), padding = (8, 8), scale = 1.05)
    
    people = []

    for box in boxes:
        x,y,width,height = box
        if height >= 80:
            people.append((int(x), int(y), int(width), int(height)))

    return people

# for this test version the logic we are goin for if there are multiple people, we will choose to go for the one closesst to the center


def choose_target(people, frame_width):
    if not people:
        return None

    image_center_x = frame_width / 2

    def distance_from_center(box):
        x, y, width, height = box
        box_center_x = x + width / 2
        return abs(box_center_x - image_center_x)
    
    return min(people, key=distance_from_center)


# okay now we will deal with the translation of the image coordinates to real nworld drone movements and shit!!!

def calculate_tracking_command(target_box, frame_width, frame_height, lidar_dist_m,target_dist_m):
    if target_box is None:
        return{
            "visible": False,
            "yaw_text": "Searching",
            "yaw_rate": 0.0,
            "forward_text": "HOLD",
            "forward_speed": 0.0,
        }
    
    x,_y,width, _height = target_box
    image_center_x = frame_width / 2
    person_center_x = x + width / 2
    x_error = person_center_x - image_center_x

    deadband_pixels = 35 # so this is a prettry interesting that allopw the drone from making microcorrections its like a zone whjere zero ncorrections happen 
    max_yaw_rate = 20.0

    if abs(x_error) <= deadband_pixels:
        yaw_text = "CENTERED"
        yaw_rate = 0.0
    else:
        yaw_rate = (x_error/image_center_x)* max_yaw_rate
        yaw_text = "YAW RIGHT" if yaw_text > 0 else "YAW LEFT"

    forward_text = "WAITING FOR LiDAR"
    forward_speed = 0.0

    if lidar_dist_m is not None:
        distance_error = lidar_dist_m - target_dist_m
        distance_deadband = 0.25
        max_forward_speed = 1.0

        if abs (distance_error) <= distance_deadband:
            forward_text = "HOLD DISTANCE"
            forward_speed = 0.0
        elif distance_error > 0:
            forward_text = "MOVE FORWARD"
            forward_speed = min(distance_error / target_dist_m, 1.0) * max_forward_speed
        else:
            forward_text = "MOVE BACKWARD"
            forward_speed = max(distance_error / target_dist_m, -1.0) * max_forward_speed

    return {
        "visible": True,
        "yaw_text": yaw_text,
        "yaw_rate": yaw_rate,
        "forward_text": forward_text,
        "forward_speed": forward_speed,
    }



