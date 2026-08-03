from pymavlink import mavutil
import time
import math

master = None
VELOCITY_YAW_MASK = 0b010111000111

def connect(connection_string="/dev/ttyAMA0", baud=57600):
    global master
    print("Connecting to Pixhawk...")
    master = mavutil.mavlink_connection(
        connection_string,
        baud=baud
    )
    master.wait_heartbeat()
    print("Heartbeat received!")
    print(

        f"System {master.target_system}, "
        f"Component {master.target_component}"
    )

def disconnect_drone():
    global master

    if master is None:
        return

    master.close()
    master = None

def get_heartbeat():
    return master.wait_heartbeat(timeout=0)

def get_attitude():
    if not is_connected():
            print("Drone not connected")
            return
    msg = master.recv_match(
        type="ATTITUDE",
        blocking=False
    )
    return msg

def get_altitude():
    if not is_connected():
            print("Drone not connected")
            return
    msg = master.recv_match(
        type="GLOBAL_POSITION_INT",
        blocking=False
    )
    if msg is None:
        return None
    return msg.relative_alt / 1000

def arm():
    if not is_connected():
        print("Drone not connected")
        return
    
    set_mode("GUIDED")
    master.arducopter_arm()
    print("Waiting for the vehicle to arm")
    master.motors_armed_wait()
    print('ARMED')

def disarm():
    if not is_connected():
            print("Drone not connected")
            return

    master.arducopter_disarm()
    master.motors_disarmed_wait()
    print("DISARMED")


def get_mode():
    if not is_connected():
            print("Drone not connected")
            return
    return master.flightmode

def set_mode(mode):
    if not is_connected():
        print("Drone not connected")
        return

    modes = master.mode_mapping()
    if mode not in modes:
        print("Unknown mode")
        return
    
    mode_id = modes[mode]
    master.set_mode(mode_id)
    while master.flightmode != mode:
        time.sleep(0.1)

def is_connected():
    return master is not None

def stop():
    hover()

def hover():
    move(
        forward_speed=0,
        right_speed=0,
        down_speed=0,
        yaw_rate=0
    )


def move(forward_speed, right_speed=0, down_speed=0, yaw_rate=0):

    if master is None:
        print("Drone not connected")
        return

    master.mav.set_position_target_local_ned_send(
    0,             
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_FRAME_BODY_NED,VELOCITY_YAW_MASK,

    0, 0, 0,
    forward_speed,
    right_speed,
    down_speed,

    0, 0, 0,

    0,
    math.radians(yaw_rate))


    print(
        f"Move | "
        f"F:{forward_speed:.2f} "
        f"R:{right_speed:.2f} "
        f"D:{down_speed:.2f} "
        f"Yaw:{yaw_rate}"
    )




    