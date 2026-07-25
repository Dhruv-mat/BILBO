import time 
# import serial

# ser = serial.Serial("/dev/serial0",115200)

def get_lidar():
    while True:
        count = ser.in_waiting
        #this part checks if atleast 8bytes of data hav been sent
        if count > 8:
            recv = ser.read(9)
            ser.reset_input_buffer()

        if recv[0] == 0x59 and recv[1] == 0x59:
            low = recv[2]
            high = recv[3]
            # basically converts it into cm
            distance = low + (high * 256)
            print(distance)
def hi():
    print('hi')

if __name__ == '__main__':
    try:
        if not ser.is_open:
            ser.open()
    except:
        if ser != None:
            ser.close()
        print("\nProgram Terminated.")
