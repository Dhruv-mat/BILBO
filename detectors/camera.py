import argparse

import cv2

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics

# COCO class index for "person". Everything else (laptop=63, etc.) is ignored.
PERSON_CLASS = 0
CONF_THRESHOLD = 0.3

last_persons = []


class Person:
    def __init__(self, x, y, width, height, confidence):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.confidence = confidence
        self.area = width * height
        self.center_x = x + width / 2
        self.center_y = y + height / 2


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=str,
        default="/usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk",
    )
    return parser.parse_args()


def parse_people(metadata):
    """Read the on-sensor inference output and return only Person detections."""
    global last_persons

    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    if np_outputs is None:
        return last_persons

    input_w, input_h = imx500.get_input_size()

    boxes = np_outputs[0][0]
    scores = np_outputs[1][0]
    classes = np_outputs[2][0]

    if intrinsics.bbox_normalization:
        boxes = boxes / input_h
    if intrinsics.bbox_order == "xy":
        # convert_inference_coords expects (y0, x0, y1, x1)
        boxes = boxes[:, [1, 0, 3, 2]]

    persons = []
    for box, score, category in zip(boxes, scores, classes):
        if int(category) != PERSON_CLASS:
            continue
        if score < CONF_THRESHOLD:
            continue
        # Returns pixel coords in the ISP output frame, handles ROI/aspect ratio.
        x, y, w, h = imx500.convert_inference_coords(box, metadata, picam2)
        persons.append(Person(x, y, w, h, float(score)))

    last_persons = persons
    return last_persons


def draw_people(request):
    """Runs in the camera thread; draws whatever the last parse produced."""
    if not last_persons:
        return

    with MappedArray(request, "main") as m:
        for p in last_persons:
            cv2.rectangle(
                m.array,
                (p.x, p.y),
                (p.x + p.width, p.y + p.height),
                (0, 255, 0),
                2,
            )
            cv2.circle(
                m.array,
                (int(p.center_x), int(p.center_y)),
                4,
                (0, 0, 255),
                -1,
            )
            cv2.putText(
                m.array,
                f"Person {p.confidence:.2f}",
                (p.x, p.y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )


def intitalise():
    global imx500, intrinsics, picam2

    args = get_args()

    imx500 = IMX500(args.model)
    intrinsics = imx500.network_intrinsics
    if not intrinsics:
        intrinsics = NetworkIntrinsics()
        intrinsics.task = "object detection"
    intrinsics.update_with_defaults()

    picam2 = Picamera2(imx500.camera_num)
    config = picam2.create_preview_configuration(
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=12,
    )

    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=True)

    picam2.pre_callback = draw_people
    frame_width, frame_height = config["main"]["size"]

    return frame_width, frame_height

def get_people():
    centers = [(p.center_x, p.center_y) for p in persons]
    persons = parse_people(picam2.capture_metadata())
    return persons
    # Coordinates of each person's center, one tuple per person this frame.
    # e.g. feed `centers` to your tracker / logger:
    # print(centers)

