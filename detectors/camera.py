"""Sony IMX500 person detection.

The model, the .rpk and the on-sensor inference path are unchanged.

Two things did change, both required for flight safety:

  1. Detections are published from picamera2's existing camera thread into a
     lock-protected slot, and the flight loop reads that slot without blocking.
     Previously the flight loop's clock was picam2.capture_metadata(), which
     blocks with no timeout -- so a camera stall froze the state machine, the
     setpoints, the LEDs and any chance of RTL, undetectably.

  2. A frame with no inference output publishes None rather than repeating the
     previous frame's detections. The original returned last_persons, so if the
     network stopped producing output the drone chased a ghost at a fixed pixel
     position forever: lost_frames never incremented, so SEARCHING and RTL never
     fired. That bypassed the exact protection the lost-frame logic exists for.
"""

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

# (monotonic timestamp, persons) for the most recent frame that actually carried
# an inference result. Updated ONLY on a real result, never on a frame that had
# none -- an inference-less frame is normal (the sensor infers at its own rate)
# and must not clobber a detection from 30 ms ago.
#
# Staleness is what makes this safe, and it is the caller's age check that
# enforces it. This is NOT the pre-review bug: that returned stale detections
# with no timestamp, no age limit and no way to notice, so the drone chased a
# frozen ghost forever. Here the timestamp belongs to the last real inference,
# so if inference stops the age grows and the caller declares a fault.
_latest = (0.0, None)

_last_frame_time = 0.0   # any callback at all: proves the ISP pipeline is alive
_callback_frames = 0
_inference_frames = 0
_parse_errors = 0


class Person:
    def __init__(self, x, y, width, height, confidence):
        # convert_inference_coords can return numpy floats, which cv2.rectangle
        # rejects. Cast once, here.
        self.x = int(x)
        self.y = int(y)
        self.width = int(width)
        self.height = int(height)
        self.confidence = float(confidence)
        self.area = self.width * self.height
        self.center_x = self.x + self.width / 2.0
        self.center_y = self.y + self.height / 2.0


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--model",
        type=str,
        default="/usr/share/imx500-models/"
                "imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk",
    )
    return parser.parse_known_args()[0]


def parse_people(metadata):
    """Return a list of Person, or None if inference output was unavailable."""
    np_outputs = imx500.get_outputs(metadata, add_batch=True)
    if np_outputs is None:
        return None

    boxes = np_outputs[0][0]
    scores = np_outputs[1][0]
    classes = np_outputs[2][0]

    input_w, input_h = imx500.get_input_size()
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
        # Reject specks: a box this small is either very distant or spurious,
        # and either way is not something to fly at.
        if person.area < cfg.MIN_BOX_AREA_PX:
            continue
        persons.append(person)

    return persons


def draw_people(request, persons):
    """Bench-only overlay. Costs real CPU and adds pipeline jitter, and nobody
    watches a preview in flight."""
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


def _on_frame(request):
    """Runs on picamera2's camera thread. Must never raise into the pipeline."""
    global _latest, _last_frame_time, _callback_frames, _inference_frames
    global _parse_errors

    persons = None
    errored = False
    try:
        metadata = request.get_metadata()
        persons = parse_people(metadata)
    except Exception:
        _log.exception("detection parse failed")
        errored = True

    now = time.monotonic()
    with _lock:
        _last_frame_time = now
        _callback_frames += 1
        if errored:
            _parse_errors += 1
        if persons is not None:
            # Only a real inference result advances the published slot and its
            # timestamp. Frames without one leave the previous result in place
            # to age out naturally.
            _latest = (now, persons)
            _inference_frames += 1

    if cfg.DEBUG_DRAW and persons:
        try:
            draw_people(request, persons)
        except Exception:
            _log.exception("overlay failed")


def intitalise():
    """Start the camera. Returns (frame_width, frame_height).

    Name retained for compatibility with existing call sites; `initialise` is
    provided below as a correctly spelled alias.
    """
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
        # The camera is mounted inverted. hflip+vflip compose to a pure 180 deg
        # rotation which cancels the mounting, so the delivered image is in true
        # world orientation. The yaw sign chain in controller.py depends on this
        # -- see cfg.YAW_PID_OUTPUT_SIGN before changing it.
        transform=Transform(hflip=True, vflip=True),
        controls={"FrameRate": intrinsics.inference_rate},
        buffer_count=12,
    )

    imx500.show_network_fw_progress_bar()
    picam2.start(config, show_preview=cfg.DEBUG_PREVIEW)

    # Detections are produced here, on picamera2's thread, so the flight loop
    # never blocks waiting for a frame.
    picam2.pre_callback = _on_frame

    frame_width, frame_height = config["main"]["size"]
    _log.info("camera started %dx%d @ %s fps",
              frame_width, frame_height, intrinsics.inference_rate)
    return frame_width, frame_height


initialise = intitalise


def get_latest():
    """Return (timestamp, persons) for the last real inference, without blocking.

    persons is None only if no inference has ever succeeded. The caller must
    treat an old timestamp as a vision fault -- the age check is what keeps
    acting on a retained detection safe.
    """
    with _lock:
        return _latest


def frame_age():
    """Seconds since any camera callback. Distinguishes a dead ISP pipeline from
    live frames whose inference has stopped."""
    with _lock:
        if _last_frame_time == 0.0:
            return float("inf")
        return time.monotonic() - _last_frame_time


def inference_alive():
    """True once at least one real inference output has been seen."""
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
        _log.exception("camera stop failed")
