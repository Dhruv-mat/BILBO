from dronekit import *


vehicle = None

def connect_drone(connection_string, waitready=True, baudrate=57600):
    global vehicle
    if vehicle == None:
        vehicle = connect(connection_string, wait_ready=waitready, baud=baudrate)
    print("drone connected")

def disconnect_drone():
    vehicle.close()


def get_ground_speed():
    return vehicle.groundspeed

def set_flight_mode(f_mode):
    global vehicle
    vehicle.mode = VehicleMode(f_mode)

def arm():
    global vehicle
    vehicle.groundspeed = 3

    print ("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print (" Waiting for vehicle to initialise...")
        time.sleep(1)

    print ("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode    = VehicleMode("STABILIZE")
    vehicle.armed   = True

    while not vehicle.armed:
        print (" Waiting for arming...")
        time.sleep(1)


def move(yaw_rate,forward_speed):
    print ("Yaw Rate:",yaw_rate)
    print("Forward Speed", forward_speed)

    