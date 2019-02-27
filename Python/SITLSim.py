import numpy as np
from StateMachine import *


class Environment:
    def propagate(self):
        assert 0, "propagate not implemented"

    def fake_imu(self):
        assert 0, "fake_imu not implemented"

    def fake_uplink(self):
        assert 0, "fake_uplink not implemented"
        
    def fake_downlink(self):
        assert 0, "fake_uplink not implemented"

    def propagate(self, initial_state, control_effort, timestep):
        # This should call orbit propagator
        print('Propagating the time forward by %.1f seconds' %timestep)
        return initial_state + 1*timestep

class SoftwareSimulation(Environment):
    def __init__(self):
        self.state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
        # The state vector [jx, jy, jz, then 4 quaternions, x, y, z, xdot, ydot, zdot]
        self.uplinkRequested = False
        self.downlinkRequested = False
        
    def requestUplink(self):
        self.uplinkRequested = True
        
    def requestDownlink(self):
        self.downlinkRequested = True

class PhysicalSat:
    def __init__(self, environment):
        self.angular_velocity = [0,0,0]
        self.battery_voltage = 10
        self.battery_beacon_threshold = 3.6 #Volts
        self.battery_tumble_threshold = 3.6 #Volts
        self.environment = environment
        self.time = 2458517.500000 #Julian date for February 3rd, 2018 at 00:00
        self.battery_payload_threshold = 3.6 #Volts
        self.payload_time = [1e7, 1e7]
        self.total_capacity = 159840 # In Joules
        
        
    def getBatteryVoltage(self):
        return self.battery_voltage
    
    def getAngularVelocity(self):
        # How are we going to estimate how the vechile perceives it's actual state?
        return self.angular_velocity

class Hardware:
    def readIMU(self):
        assert 0, "readImu not implemented"

    def runMagnetorquer(self, m_value):
        assert 0, 'runMagnetorquer not implemented'

    def checkBatteryPercent(self):
        assert 0, 'checkBatteryPercent not implemented'



class SoftwareSimHardware(Hardware):
    """Class to represent the hardware in the Software in the Loop Simulation

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
        """Class to represent the hardware in the Software in the Loop Simulation

        Assumptions:
        Battery starts at full capacity

        Source:
        N/A

        Inputs:
        None

        Output:
        None

        Properties Used:
        None
        """
        self.max_battery_capacity = 159840 #In J
        self.uplink_requested = False
        self.downlink_requested = False
        self.time = 0 #should put this in julian date....?
        self.payload_time = [1e7]

        # Physical properties
        # Define Principle Axes of Inertia
        J11 = .01/6              # (kg m^2)
        J22 = .01/6              # (kg m^2)
        J33 = .01/6              # (kg m^2)
        self.J = np.diag(np.array([J11,J22,J33])) # Diagonalize Principle Axes
        self.J_inv = np.linalg.inv(self.J) # Invert J Matrix for Ease of Computation

        # Initialize the Magnetorquer Properties
        area_coil = np.array([0.8784,0.8784,0.8784])   # Magnetic area of coils (m sq)
        voltage_max = 8.4                              # max voltage to coils (volts)
        resistance  = np.array([178.4,178.4,135])      # resistance of coils (ohm)
        I_max = np.divide(voltage_max,resistance)      # Maximum current (A)
        m_max = I_max*area_coil                        # Maximum magnetic moment (A.msq)
        self.m_max = np.reshape(m_max,(1,3))
        self.power_max = voltage_max*I_max                  # Max power consumed (W)


    def getHardwareProperties(self):
        return (self.J, self.J_inv, self.m_max, self.power_max)

    def readIMU(self):
        """Returns the values the IMU would read.

        Assumptions:
        accelerations aren't used and can be set to zero

        Source:
        N/A

        Inputs:
        None

        Output:
        9 element tuple in order of accelerations, rotation velocities, magnetic field readings
        in the body frame

        Properties Used:
        None
        """   

        # Obvious placeholder. 
        return (0, 0, 0, 0, 0, 0, 0, 0, 0)

    def checkBatteryPercent(self):
        """Returns the values battery current percentage

        Assumptions:
        Percent is just current_value/max_value

        Source:
        N/A

        Inputs:
        None

        Output:
        Battery remaining percentage

        Properties Used:
        None
        """

        #placeholder value
        return 50

    def runMagnetorquer(self, m_value):
        """Sets the magnetorquer moment value

        Assumptions:
        WHAT ARE THE UNITS OF m_value?!?!?

        Source:
        N/A

        Inputs:
        m_value

        Output:
        None

        Properties Used:
        None
        """

        #THIS SHOULD SET A VALUE IN THE SIMULATION PROPAGATE OR SOMETHING


# Static variable initialization:
PandaSat.hold = Hold()
PandaSat.deploy_antenna = DeployAntenna()
PandaSat.tumble_check = TumbleCheck()
PandaSat.sleep = Sleep()
PandaSat.battery_tumble_check = BatteryTumbleCheck()
PandaSat.detumble = Detumble()
PandaSat.battery_beacon_check = BatteryBeaconCheck()
PandaSat.beacon = Beacon()
PandaSat.listen = Listen()
PandaSat.uplink_check = UplinkCheck()
PandaSat.process_uplink = ProcessUplink()
PandaSat.downlink_check = DownlinkCheck()
PandaSat.downlink = Downlink()
PandaSat.payload_schedule_check = PayloadScheduleCheck()
PandaSat.battery_payload_check = BatteryPayloadCheck()
PandaSat.payload_on = PayloadOn()

def initialize_orbit():
    # Orbital Elements and Parameters
    a = 6917.50              # Orbit Semi-Major Axis (km)
    e = 0.000287             # Eccentricity
    i = 28.47                # Inclination (deg)
    RAAN = 176.23            # Right Ascension of Ascending Node (deg)
    w = 82.61                # Argument of Perigee (deg)
    anom = 319.41            # Mean Anomaly (deg)
    mu = 3.986e5             # Earth Standard Gravitational Parameter (km^3/s^2)
    T = 2*np.pi*np.sqrt((a**3)/mu) # Orbital Period (s)
    r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)

def runSimulationTime(state_machine, max_time):
    # Assumptions
    # time starts at zero

    J, J_inv, m_max, power_max = ps.hardware.getHardwareProperties()
    time = 0
    while time <= max_time:
        delta_t, _ = state_machine.runStep(None)
        time += delta_t
        print('current time: %.1f min' %(time/60))

def runSimulationSteps(state_machine, num_steps):
    # Assumptions
    # time starts at zero

    J, J_inv, m_max, power_max = ps.hardware.getHardwareProperties()
    time = 0
    for i in range(num_steps):
        delta_t, _ = state_machine.runStep(None)
        time += delta_t
        print('current time: %.1f min' %(time/60))


#Simulation


#orbit_sim = SoftwareSimulation()
hardware = SoftwareSimHardware()
#inputs = (None,None,None,None,None, None, None, None, None)
ps = PandaSat(hardware)
runSimulationSteps(ps, 11)
#ps.runAll(inputs)
print('\n')
#orbit_sim.requestUplink()
#inputs2 = (None,None,None,None,None, None, None, None)
#ps.runAll(inputs2)