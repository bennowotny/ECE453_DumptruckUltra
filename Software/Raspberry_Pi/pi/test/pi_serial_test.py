import serial
import time

if __name__=='__main__':
    ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)
    ser.reset_input_buffer()        # flush buffer

    while True:
        ser.write(b"Sending from the PI:\r\n");
        # line = ser.readline().decode('utf-8').rstrip()
        # print(line)
        time.sleep(1)
        '''
        if ser.in_waiting>0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        '''
