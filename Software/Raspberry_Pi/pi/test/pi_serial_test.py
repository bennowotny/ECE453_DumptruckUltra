import serial
import time
import struct

if __name__=='__main__':
    ser = serial.Serial('/dev/ttyAMA0', 115200)
    ser.reset_input_buffer()        # flush buffer 
    while True:
        # ser.write(b"Sending from the PI:\r\n");
        results_string = b''
        results_string += struct.pack('f',1.0)
        results_string += struct.pack('f',1.0)
        ser.write(results_string)
        # ser.write(bytes(1));
        # line = ser.readline().decode('utf-8').rstrip()
        # print(line)
    
        
        [x] = struct.unpack('f',results_string[0:4])
        [y] = struct.unpack('f',results_string[4:8])
        print(f'Sending data {x} {y}')
        time.sleep(1)
        '''
        if ser.in_waiting>0:
            line = ser.readline().decode('utf-8').rstrip()
            print(line)
        '''
