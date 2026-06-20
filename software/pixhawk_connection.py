import argparse
import time

def connect_to_pixhawk(connection_string,baudrarte):

    from dronekit import connect_to_pixhawk

    print(f"Connecting to pixhawk on {connection_string}")

    vehicle = connect(connection_string,wait_read=True, baud = baudrarte)
    print("connected to piuxhawk")


# okay so this par tnow is gonna be the data im gonna need for reference purpose only, basically pixhawk telem data

def print_vehicle_data(vehicle):
    print("Mode:", vehicle.mode.name)
    print("Armed:", vehicle.armed)
    print("EKF Ok:", vehicle.ekf_ok)
    print("Battery :", vehicle.battery)
    print("GPS:", vehicle.gps_0)
    print("Location:", vehicle.location.global_relative_frame)
    print("Velocity:", vehicle.velocity)
    print("Heading:", vehicle.heading)


def close_pixhawk(vehicle):

    vehicle.close()
    print("pixhawk connection ended")

def main():

    parser = argparse.ArgumentParser(description="Connecting to Pixhawk")

    parser.add_argument("--port", default="/dev/ttyACM1", help="Pixhawk connection string")
    parser.add_argument("--baud", type=int, default=57600, help="Serial Baud Rate")
    parser.add_argument("--repeat", type=int, default=10, help="No. of Readings")
    parser.add_argument("--delay", type=float, default=1.0, help="Seconds btw readings")
    args = parser.parse_args()

    vehicle = connect_to_pixhawk(args.connect,args.baud)

    try: 
        for _ in range(args.repeat):
            print_vehicle_data(vehicle)
            time.sleep(args.delay)

    finally:
        close_pixhawk(vehicle)

if __name__ == "__main__":
    main()

    


