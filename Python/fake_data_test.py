import serial
import io


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
import serial
import time

ser = serial.Serial('/dev/cu.usbserial-1450', 115200, timeout = 1) # ttyACM1 for Arduino board

readOut = 0   #chars waiting from laser range finder

print ("Starting up")
connected = False
commandToSend = 'help' # get the distance in mm
count = 0
while count < 5:
    print ("Writing: ",  commandToSend)
    ser.write(str(commandToSend).encode())
    time.sleep(1)
    while True:
        try:
            print ("Attempt to Read")
            readOut = ser.readline().decode('ascii')
            time.sleep(1)
            print ("Reading: ", readOut) 
            break
        except:
            pass
    print ("Restart\n")
    ser.flush() #flush the buffer
    count += 1

