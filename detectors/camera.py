import argparse
import logging
import threading
import time

import cv2

from picamera2 import MappedArray, Picamera2
from picamera2.devices import IMX500
from picamera2.devices.imx500 import NetworkIntrinsics
from libcamera import Transform

import config as cfg

_log = logging.getLogger(__name__)

imx500 = None
intrinsics = None
picam2 = None
_lock = threading.Lock()
_latest = (0.0, None)
_last_frame_time = 0.0   
_callback_frames = 0
_inference_frames = 0
_parse_errors = 0

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
    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    if np_outputs is None:
        return None

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
        if int(category) != cfg.PERSON_CLASS:
            continue
        if score < cfg.CONF_THRESHOLD:
            continue
        x, y, w, h = imx500.convert_inference_coords(box, metadata, picam2)
        person = Person(x, y, w, h, score)
        if person.area < cfg.MIN_BOX_AREA_PX:
            continue
        persons.append(person)

    return persons


def draw_people(request, persons):
    if not persons:
        return
    with MappedArray(request, "main") as m:
        for p in persons:
            cv2.rectangle(m.array, (p.x, p.y),
                          (p.x + p.width, p.y + p.height), (0, 255, 0), 2)
            cv2.circle(m.array, (int(p.center_x), int(p.center_y)), 4,
                       (0, 0, 255), -1)
            cv2.putText(m.array, "Person %.2f" % p.confidence,
                        (p.x, p.y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)


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
       
        transform=Transform(hflip=True, vflip=True),
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=12,
    )

    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=cfg.DEBUG_PREVIEW)

    picam2.pre_callback = _on_frame

    frame_width, frame_height = config["main"]["size"]
    _log.info("camera started %dx%d @ %s fps",
              frame_width, frame_height, intrinsics.inference_rate)
    return frame_width, frame_height

initialise = intitalise

def get_latest():
    with _lock:
        return _latest

def frame_age():
    with _lock:
        if _last_frame_time == 0.0:
            return(float("inf"))
        return time.monotonic() - _last_frame_time



def infrence_alive():
    with _lock:
        return _inference_frames > 0

def health():
    with _lock:
        return {
            "frames": _callback_frames,
            "inferences": _inference_frames,
            "parse_errors": _parse_errors,
        }


def stop():
    if picam2 is None:
        return
    try:
        picam2.pre_callback = None
        picam2.stop()
    except Exception:
        _log.exception("camera stoped and failed")
