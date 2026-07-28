import time
import serial
import math



ser = serial.Serial("/dev/ttyAMA0", 115200)

def read_data():
    counter = ser.in_waiting # count the number of bytes of the serial port
    if counter > 8:
        bytes_serial = ser.read(9)
        ser.reset_input_buffer()

        if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
            distance = bytes_serial[2] + bytes_serial[3]*256 
            ser.reset_input_buffer()
            return distance






if __name__ == "__main__":
    try:
        if ser.isOpen() == False:
            ser.open()
        read_data()
    except KeyboardInterrupt(): # ctrl + c in terminal.
        if ser != None:
            ser.close()
            print("program interrupted by the user")