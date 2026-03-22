import depthai as dai
import cv2
import numpy as np

pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
monoLeft = pipeline.create(dai.node.MonoCamera)
monoRight = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)
faceDet = pipeline.create(dai.node.MobileNetDetectionNetwork)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutDepth = pipeline.create(dai.node.XLinkOut)
xoutDet = pipeline.create(dai.node.XLinkOut)

xoutRgb.setStreamName("rgb")
xoutDepth.setStreamName("depth")
xoutDet.setStreamName("det")

camRgb.setPreviewSize(300, 300)
camRgb.setInterleaved(False)

monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)

monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

faceDet.setBlobPath("face-detection-retail-0004.blob")
faceDet.setConfidenceThreshold(0.5)

camRgb.preview.link(faceDet.input)
camRgb.preview.link(xoutRgb.input)

monoLeft.out.link(stereo.left)
monoRight.out.link(stereo.right)
stereo.depth.link(xoutDepth.input)

faceDet.out.link(xoutDet.input)

with dai.Device(pipeline) as device:
    qRgb = device.getOutputQueue("rgb", 4, False)
    qDepth = device.getOutputQueue("depth", 4, False)
    qDet = device.getOutputQueue("det", 4, False)

    while True:
        frame = qRgb.get().getCvFrame()
        depthFrame = qDepth.get().getFrame()
        detections = qDet.get().detections

        h, w = frame.shape[:2]

        for det in detections:
            x1 = int(det.xmin * w)
            y1 = int(det.ymin * h)
            x2 = int(det.xmax * w)
            y2 = int(det.ymax * h)

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            depth = depthFrame[cy][cx]

            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)

            print("Face:", cx, cy, "Depth:", depth)

        depthVis = cv2.normalize(depthFrame, None, 255, 0, cv2.NORM_INF, cv2.CV_8UC1)
        depthVis = cv2.applyColorMap(depthVis, cv2.COLORMAP_JET)

        cv2.imshow("rgb", frame)
        cv2.imshow("depth", depthVis)

        if cv2.waitKey(1) == ord('q'):
            break