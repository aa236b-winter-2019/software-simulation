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
print('Waiting for packets...')
while True:
    packet = rfm9x.receive()
    # Optionally change the receive timeout from its default of 0.5 seconds:
    #packet = rfm9x.receive(timeout_s=5.0)
    # If no packet was received during the timeout then None is returned.
    if packet is None:
        print('Received nothing! Listening again...')
    else:
        # Received a packet!
        # Print out the raw bytes of the packet:
        print('Received (raw bytes): {0}'.format(packet))
        # And decode to ASCII text and print it too.  Note that you always
        # receive raw bytes and need to convert to a text format like ASCII
        # if you intend to do string processing on your data.  Make sure the
        # sending side is sending ASCII data before you try to decode!
        packet_text = str(packet, 'ascii')
        print('Received (ASCII): {0}'.format(packet_text))
        # Also read the RSSI (signal strength) of the last received message and
        # print it.
        rssi = rfm9x.rssi
        print('Received signal strength: {0} dB'.format(rssi))