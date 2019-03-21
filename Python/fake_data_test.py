import serial
import io
from CPsubroutines import *

'''
feather_port = '/dev/cu.usbserial-1450' #this is the computer to USB/Serial converter serial port

ser = serial.Serial(feather_port)  # open serial por
sio = io.TextIOWrapper(io.BufferedRWPair(ser, ser))
sio.write("hello\n")
sio.flush() # it is buffering. required to get the data out *now*
test = sio.readline()
print(hello == b"hello\n")
print(finished)

print(ser.name)         # check which port was really used
ser.write(b'hello\n')     # write a string
#ser.close()             # close port
count = 0
while count < 10:
	line = ser.readline()
	#print(str(count) + str(': ') + chr(line))
	print(chr(line))
	count += 1

ser.close()
'''

#ls /dev/tty.* how to check a mac's serial ports
import serial
import time
import math
import numpy as np

ser = serial.Serial('/dev/cu.usbserial-1440', 57600, timeout = 1) # ttyACM1 for Arduino board

#nreadOut = 0   #chars waiting from laser range finder

# fakeIMU = ([0, 0, 0], [1.45, 1.45, 1.45], [3.45, 3.45, 3.45])

print ("Starting up")


omega = np.rad2deg([ 0.24917651, -0.24928519, -0.12525748])
B_Body = [1.31552664e-05, 2.07796739e-05, 1.70934904e-05]
command_to_write = encodestring(([1.2, 1.2, 1.2], omega, B_Body, 2, 5), 6)

writeToSerial(ser, command_to_write)
message_out = readFromSerial(ser)


#writeToSerial(([3.1, 3.2, 3.3], [2.1, 2.1, 2.1], [3.4, 3.4, 3.4], 2, 5), 2)

#writeToSerial(([3.1, 3.2, 3.3], [2.1, 2.1, 2.1], [3.4, 3.4, 3.4], 2, 5), 2)


'''
count = 0
while count < 1:
    print ("Writing: ",  commandToSend)
    #ser.write('IMU Data\n'.encode('utf-8'))
    ser.write(str(commandToSend).encode('utf-8'))
    #ser.write(commandToSend.encode('utf-8'))
    time.sleep(2)
    while True:
        try:
            print ("Attempt to Read")
            raw = ser.readline()

            #data_string = ''.join([chr(b) for b in readOut])
            readOut = ''.join([chr(b) for b in raw])
            time.sleep(2)
            #print('Raw output: ')
            print ("Reading: ", raw)
            break
        except:
            pass
    print ("Restart\n")
    ser.flush() #flush the buffer
    count += 1

test = encodestring(([1, 1, 1], [2, 2, 2], [3, 3, 3], 2, 5), 5)

test2 = decodestring(test)

print(test)
'''
