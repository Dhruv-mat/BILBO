import argparse
from email import parser
import cv2
import torch

def create_person_detector():
    from ultralytics import YOLO
    return YOLO("yolov8n.pt")

def find_people(detector, frame,confidence):

    people = []
    results = detector.predict(frame,classes =[0],conf=confidence,verbose=False)
    
    if not results:
        return people
    
    result = results[0]

    if result.boxes is None:
        return people
    for detected_box in result.boxes:
        x1,y1,x2,y2 = detected_box.xyxy[0].cpu().tolist()
        x = int(x1)
        y = int(x2)
        width = int(x2-x1)
        height = int(y2-y1)

        if width > 0 and height > 0:
            people.append((x,y,width,height))

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
        yaw_text = "YAW RIGHT" if yaw_rate > 0 else "YAW LEFT"

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

def draw_output(frame, people, target_box, command, lidar_dist_m): #this part is just the drawing opencv boixes stuff and text on the screen
    frame_height, frame_width = frame.shape[:2]
    image_center = (frame_width // 2, frame_height // 2)

    # smth to add some good visuals like its just some bounding box colour changing and stuff like that
    for box in people:
        x, y, width, height = box
        colour = (80,80,80)
        if box == target_box:
            colour = (0, 255, 0)
        cv2.rectangle(frame, (x, y), (x + width, y + height), colour, 2)

    cv2.circle(frame, image_center, 5, (0, 0, 255), -1)

    if target_box is not None:
        x, y, width, height = target_box
        person_center = (x + width // 2, y + height // 2)
        cv2.circle(frame, (person_center), 8, (0, 0, 255), -1)
        cv2.line(frame, image_center, person_center, (255, 0, 0), 2)

    range_text = "LiDAR: not connected"
    if lidar_dist_m is not None:
        range_text = f"LiDAR: {lidar_dist_m:.2f} m"

    cv2.rectangle(frame, (20, 20), (420,160), (30, 30, 30), -1)
    cv2.rectangle(frame, (20, 20), (420,160), (0, 255, 255), 2)
    cv2.putText(frame, command['yaw_text'], (40, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(frame, command['forward_text'], (40, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(frame, range_text, (40, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)

    cv2.putText(frame, f"Yaw Rate: {command['yaw_rate']:+.1f} deg/s | Forward Speed: {command['forward_speed']:+.2f} m/s", (20, frame_height - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

    return frame

#thinking????

#ohhhh ass i forgot to open the camer lmaoooooo

def open_camera(camera_index):
    camera = cv2.VideoCapture(camera_index)
    if not camera.isOpened():
        raise RuntimeError(f"Failed to open camera") #look at me being all formal and shit :-)
    return camera

#okay now we are gonna be building the main thing, buit imma put some terminal things cause this a testfile and i will lowkey find it helpfull if some certain configs are shiown to me at the terminal window

def main():

    parser = argparse.ArgumentParser(description="Person Tracking")
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--lidar", type=float, default=2.0, help="LiDAR distance")
    parser.add_argument("--fake-lidar", type=float, default=None, help="Use fake LiDAR for testing")
    args = parser.parse_args()

    detector = create_person_detector()
    camera = open_camera(args.camera)

    try: 
        while True:
            success, frame = camera.read()
            if not success:
                raise RuntimeError("Failed to read frame from camera")
                break

            frame_height, frame_width = frame.shape[:2]
            people = find_people(detector, frame,0.35)
            target_box = choose_target(people, frame_width)
            command = calculate_tracking_command(target_box, frame_width, frame_height, args.fake_lidar, args.lidar)

            output = draw_output(frame, people, target_box, command, args.fake_lidar)
            cv2.imshow("Person Tracking", output)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()
if __name__ == "__main__":
    main()
        