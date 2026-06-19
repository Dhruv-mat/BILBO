import argparse
import time

# go for a lower baudrate if the tf luna is not responding at this current value
def connect_to_lidar(port, baudrate=115200):

    import serial 

    print(f"Connecting lidar to port {port}")
    serial_port =serial.Serial(port,baudrate,timeout=0)

    if not serial_port.isOpen():
        serial_port.open()

    print("connected to LiDAR")
    return serial_port


#basically this reades on packet of the data that comes out of the tf luna
def read_lidar_distance(serial_port):

    while True:
        bytes_waiting =serial_port.in_waiting
        if bytes_waiting <= 6:
            continue

        packet = serial_port.read(7)
        serial_port.reset_input_buffer()

        pckt_has_valid_header = packet[0] = 0x59 and packet[1] == 0x59

        if not pckt_has_valid_header:
            continue

        dist_cm = packet[2] + packet[3] * 256
        signal_strength = packet[4] + packet[5] *256
        distance_m = dist_cm/100.0
        return distance_m, signal_strength
    
def close_lidar(serial_port):

    print("Lidar Hashake ending")
    serial_port.close()
    print("Lidar connection terminated")

def main():
    parser = argparse.ArgumentParser(description="Reading Distance from TF LUNA")

    parser.add_argument("--port", default="/dev/ttyTHS1", help="TF Luna Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="TF Luna Baud Rate")
    parser.add_argument("--repeat", type=int, default=10, help="No. of Readings")
    parser.add_argument("--DELAY", type=float, default=0.1, help="Seconds btw readings")
    args = parser.parse_args()

    serial_port = connect_to_lidar(args.port,args.baud)
    try:
        for _ in range(args.repeat):
            distance_m, strength = read_lidar_distance(serial_port)
            print(f"Distance: {distance_m:.2f} m | Strength: {strength}")
            time.sleep(args.delay)

    finally:
        close_lidar(serial_port)

if __name__ == "__main__":
    main()