## Code to send messages via rfm9x radio

import board
import busio
import digitalio
import adafruit_rfm9x

# Define radio parameters.
RADIO_FREQ_MHZ   = 433.0  # Frequency of the radio in Mhz

# Define pins connected to the chip:
CS    = digitalio.DigitalInOut(board.D5)
RESET = digitalio.DigitalInOut(board.D6)

# Initialize SPI bus.
spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Initialze RFM radio
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)
while True:
    rfm9x.send('Hello world!\r\n')
    print('Sent hello world message!')