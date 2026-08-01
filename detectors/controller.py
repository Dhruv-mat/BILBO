import drone
from simple_pid import PID

MAX_YAW_RATE =20
YAW_DEADBAND = 50
TARGET_DISTANCE = 160  
MAX_YAW_RATE = 15    
MAX_FORWARD_SPEED = 2

yaw_pid = PID(
    Kp=0.10,
    Ki=0.00,
    Kd=0.00,
    setpoint=0
)

distance_pid = PID(
    Kp=0.02,
    Ki=0.00,
    Kd=0.00,
    setpoint=TARGET_DISTANCE
)

yaw_pid.output_limits = (-MAX_YAW_RATE, MAX_YAW_RATE)

distance_pid.output_limits = (-MAX_FORWARD_SPEED, MAX_FORWARD_SPEED)

def update(error_x, distance):

    if abs(error_x) < YAW_DEADBAND:
        yaw_rate = 0
    else:
        yaw_rate = yaw_pid(error_x)

    print(f"Distance: {distance}")
    
    if distance is None:
        forward_speed = 0
    else:
        forward_speed = -distance_pid(distance)
    drone.move(
        yaw_rate=yaw_rate,
        forward_speed=forward_speed
    )