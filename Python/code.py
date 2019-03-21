from CPsubroutines_cp import *
import board
import busio
import digitalio
import time
import math

def calcMValue(imu_reading):
    m_max = .009626
    om0 = [axis_reading * (math.pi/180.0) for axis_reading in imu_reading[1]]
    #om0 = imu_reading[1]*(math.pi/180.0)

    #Get B Vector in gauss and normalize it
    B_body = [axis_reading * 10**-4 for axis_reading in imu_reading[2]]

    B_body_norm = math.sqrt(B_body[0]**2 + B_body[1]**2  + B_body[2]**2 )

    B_body = [axis_reading * (1/B_body_norm) for axis_reading in B_body]
    #B_body = imu_reading[2]*10**-4

    m_value=[]
    B_dot1 = (om0[1]*B_body[2]-om0[2]*B_body[1])               # Compute B_dot
    B_dot2 = -(om0[0]*B_body[2]-om0[2]*B_body[1])               # Compute B_dot
    B_dot3 = (om0[0]*B_body[1]-om0[1]*B_body[0])               # Compute B_dot


    #tanh1=hypertan(om0[0])
    #tanh2=hypertan(om0[1])
    #tanh3=hypertan(om0[2])
    #tanh_om=math.sqrt(tanh1**2+tanh2**2+tanh3**2)

    m_value.append(math.copysign(m_max, B_dot1))
    m_value.append(math.copysign(m_max, B_dot2))
    m_value.append(math.copysign(m_max, B_dot3))

    print(m_value)
    return(m_value)

led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

uart = busio.UART(board.TX, board.RX, baudrate=57600)
data = None
while True:
    #print('slow')
    #print(bytes('from c', 'utf-16'))
    #uart.write(bytes('from c', 'utf-8'))
    #uart.write('test\n')
    data = uart.readline()  # read up to 32 bytes
    # print(data)  # this is a bytearray type

    if data is not None:
        #led.value = True



        # convert bytearray to string
        data_string = ''.join([chr(b) for b in data])
        #print(data_string, end="")
        print('\n')
        print(decodestring(data_string))

        imu_reading = decodestring(data_string)
        #print(a[0][0] + a[0][1])
        #print(data.decode('utf-8'))

        #led.value = False
        m_value = calcMValue(imu_reading)

        command_to_send = encodestring((m_value, [0,0,0], [0,0,0], 0, 0), 5)
        uart.write(bytes(command_to_send + '\n', 'utf-8'))
        #uart.write('test\n')

print('Escaped!')


#imu_reading = hardware.readIMU()



#calcMValue(imu_reading)

'''
import board
import busio
import digitalio

led = digitalio.DigitalInOut(board.D13)
led.direction = digitalio.Direction.OUTPUT

uart = busio.UART(board.TX, board.RX, baudrate=9600)

while True:
    data = uart.read(32)  # read up to 32 bytes
    # print(data)  # this is a bytearray type

    if data is not None:
        led.value = True

        # convert bytearray to string
        data_string = ''.join([chr(b) for b in data])
        datastr = ''.join([chr(b) for b in data]) # convert bytearray to string
        print(data_string, end="")

        led.value = False
'''
'''
# Simple demo of the LSM9DS1 accelerometer, magnetometer, gyroscope.
# Will print the acceleration, magnetometer, and gyroscope values every second.
import time
import board
import busio
import adafruit_lsm9ds1

# I2C connection:
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_lsm9ds1.LSM9DS1_I2C(i2c)

#SPI connection:
# from digitalio import DigitalInOut, Direction
# spi = busio.SPI(board.SCK, board.MOSI, board.MISO)
# csag = DigitalInOut(board.D5)
# csag.direction = Direction.OUTPUT
# csag.value = True
# csm = DigitalInOut(board.D6)
# csm.direction = Direction.OUTPUT
# csm.value = True
# sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)

# Main loop will read the acceleration, magnetometer, gyroscope, Temperature
# values every second and print them out.
while True:
    # Read acceleration, magnetometer, gyroscope, temperature.
    accel_x, accel_y, accel_z = sensor.acceleration
    mag_x, mag_y, mag_z = sensor.magnetic
    gyro_x, gyro_y, gyro_z = sensor.gyro
    temp = sensor.temperature
    # Print values.
    print('Acceleration (m/s^2): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (gauss): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(mag_x, mag_y, mag_z))
    print('Gyroscope (degrees/sec): ({0:0.3f},{1:0.3f},{2:0.3f})'.format(gyro_x, gyro_y, gyro_z))
    print('Temperature: {0:0.3f}C'.format(temp))
    # Delay for a second.
    time.sleep(2.0)
'''