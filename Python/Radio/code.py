## Code to send and recieve messages via rfm9x radio

import board
import busio
import digitalio
import adafruit_rfm9x

class Radio(Hardware):
	"""Class to use the radio to send and recieve signals using RFM9x radio, during hardware in the Loop Simulation
    runs on the micro controller

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    None

    Output:
    None

    Properties Used:
    None
    """

    def __init__(self):
    	self.send_msg = None
		RADIO_FREQ_MHZ   = 433.0  # Frequency of the radio in Mhz

		# Define pins connected to the chip:
		CS    = digitalio.DigitalInOut(board.D5)
		RESET = digitalio.DigitalInOut(board.D6)

		# Initialize SPI bus.
		spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

		# Initialze RFM radio
		self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

	def send_signals(self):
		# Define radio parameters.
		self.rfm9x.send(self.send_msg)
		print('message sent')

		return

	def recieve_signals(self):
		print('Waiting for packets...')
		while True:
		    packet = self.rfm9x.receive()
		    # Optionally change the receive timeout from its default of 0.5 seconds:
		    #packet = rfm9x.receive(timeout_s=5.0)
		    # If no packet was received during the timeout then None is returned.
		    if packet is None:
		        print('Received nothing! Listening again...')
		    else:
		        # Received a packet!
		        # Print out the raw bytes of the packet:
		        print('Received (raw bytes): {0}'.format(packet))
		        # decode to ASCII text 
		        packet_text = str(packet, 'ascii')
		        print('Received (ASCII): {0}'.format(packet_text))
		        #read the RSSI (signal strength) of the last received message
		        # rssi = rfm9x.rssi
		        # print('Received signal strength: {0} dB'.format(rssi))
		        return packet_text