import torch
import cv2

model = torch.hub.load('ultralytics/yolov5', 'yolov5n', pretrained=True)

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)

    for *box, conf, cls in results.xyxy[0]:
        if int(cls) == 0:  # person class
            x1, y1, x2, y2 = map(int, box)

            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)
            cv2.circle(frame, (cx, cy), 5, (0,0,255), -1)

            print("center:", cx, cy)

    cv2.imshow("frame", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()