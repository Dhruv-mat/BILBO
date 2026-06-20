# How To Run This Code

## 1. Pixhawk Connection

File: 

'''text 
pixhawk_connection.py
'''

Run on the Jetson nano with the pixhawk connceted


## 2. LiDAR Reader

File: 

'''text 
lidar_reader.py
'''

Run on the Jetson when the LiDAR is connected

It will print the distance in meters and signal strenght

## 3. Person Tracking Demo

File: 

'''text 
person_tracking.py
'''

This file uses YOLO through the 'ultralytics' library. The defautlt model is 
'yolov8n.pt' which is a small YOLOv8 nano model.

press 'q' to quit the file

To simulate the LiDAR distance temporaily the code rn has the LiDAR values
hard codded. '3.0 m' is too far and '1.0 m' is too close.

## How the entire thing works

File: 

'''text 

1. Camera detects the person
2. Code finds the persons center
3. Person center is commpared to image center
4. Yaw left or right is then calculated
5. LiDAR distance is then compared to the desired distance
6. Forward or Backward is then determined
7. A Command is then sent to the pixhawk

'''