## Code to send and recieve messages via rfm9x radio
import Hardware_class
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
    	self.recieved_msg = None
		RADIO_FREQ_MHZ   = 433.0  # Frequency of the radio in Mhz

		# Define pins connected to the chip:
		CS    = digitalio.DigitalInOut(board.D5)
		RESET = digitalio.DigitalInOut(board.D6)

		# Initialize SPI bus.
		spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

		# Initialze RFM radio
		self.rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ)

	def send_signals(self):
		self.rfm9x.send(str(self.send_msg))
		return

	def recieve_signals(self):
		print('Waiting for packets...')
	    packet = self.rfm9x.receive()
	    if packet is None:
	        self.recieved_msg = None
	    else:
	        self.recieved_msg = str(packet, 'ascii')
	    return 