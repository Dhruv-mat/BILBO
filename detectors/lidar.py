import time
import serial
import math



ser = serial.Serial("/dev/ttyAMA0", 115200)

def read_data():
    if ser.in_waiting >= 9:
        packet = ser.read(9)

        if packet[0] == 0x59 and packet[1] == 0x59:
            return packet[2] + (packet[3] << 8)

    return None






if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        read_data()
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser != None:
            ser.close()
            print("program interrupted by the user")