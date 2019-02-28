import numpy as np
from StateMachine import *
from igrffx import igrffx
from dynamics import dynamics
import julian
from propagate import propagate
from subroutines import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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

        # Initial time in MJD
        self.time = 54372.78 # MJD

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
        self.m_value = [0, 0, 0] #maqnetorquer initially off

        self.initialize_orbit()
        aaaa = 1


    def getHardwareProperties(self):
        return (self.J, self.J_inv, self.m_max, self.power_max)

    def readIMU(self):
        """Returns the values the IMU would read.

        Assumptions:
        accelerations aren't used and can be set to zero
        Currently no noise

        Source:
        N/A

        Inputs:
        None

        Output:
        3 3-element tuples in order of accelerations, rotation velocities, magnetic field readings
        in the body frame

        Properties Used:
        None
        """

        rv_eci = self.state[0:6]                      # Initial Orbit State Vector
        omq = self.state[6:]                          # Initial Angular Velocity (rad/s) and Quaternion 


        # Calculate Earth's Magnetic Field in ECI
        t0 = julian.from_jd(self.time, fmt='mjd')            # Convert mjd into seconds
        B_eci = igrffx(rv_eci[0:3],t0)*10**-9

        q = omq[3:7]

        # Rotate B_eci into Satellite Body Frame
        B_body = np.dot(q2rot(q),B_eci)   

        return ([0, 0, 0], omq[0:3], B_body)

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
        

        Source:
        N/A

        Inputs:
        m_value A/m^2

        Output:
        None

        Properties Used:
        None
        """

        self.m_value = m_value

        #THIS SHOULD SET A VALUE IN THE SIMULATION PROPAGATE OR SOMETHING

    def initialize_orbit(self):
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

        # Initialize Known Variables and Initial Conditions
        rv_eci0 = np.append(r_eci, v_eci)   # Initial Orbit State Vector
        om = 0.25*np.array([1,1,1])         # Initial Angular Velocity (rad/s)
        q = np.array([0,0,1,0])             # Initial Quaternion, Identity
        torque = np.array([0,0,0])          # Initial Torque
        power = np.array([0])               # Initial Power
        omqtp0 = np.concatenate((om,q,torque,power)) # Initial Attitude State Vector  

        self.state = np.concatenate((rv_eci0, omqtp0))


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



    

def runSimulationTime(state_machine, max_time):
    # Assumptions
    # time starts at zero

    J, J_inv, m_max, power_max = ps.hardware.getHardwareProperties()
    time = 0
    while time <= max_time:
        delta_t, _ = state_machine.runStep(None)
        time += delta_t
        state_machine.hardware.time += (delta_t/86400)
        print('current time: %.1f min' %(time/60))

def runSimulationSteps(state_machine, num_steps):
    # Assumptions
    # time starts at zero
    xyz_hist = np.zeros((num_steps, 3))
    w_hist = np.zeros((num_steps, 3))
    time_hist = np.zeros(num_steps)


    time = 0
    for i in range(num_steps):
        xyz_hist[i, :] = state_machine.hardware.state[0:3]
        w_hist[i,:] = state_machine.hardware.state[6:9]
        time_hist[i] = time
        delta_t, _ = state_machine.runStep(None)
        state_machine.hardware.state = propagate(state_machine.hardware, delta_t)
        state_machine.hardware.time += (delta_t/86400)


        time += delta_t
        #print('current time: %.1f min' %(time/60))

    return (xyz_hist, time_hist, w_hist)


#Simulation


#orbit_sim = SoftwareSimulation()
hardware = SoftwareSimHardware()
#inputs = (None,None,None,None,None, None, None, None, None)
ps = PandaSat(hardware)
xyz_hist, time_hist, w_hist = runSimulationSteps(ps, 6000)
#print(w_hist)

# Plot Orbit 
plt.figure()
ax=plt.axes(projection='3d')
ax.plot3D(xyz_hist[:,0],xyz_hist[:,1],xyz_hist[:,2],'r')
ax.axis('equal')
plt.xlabel('[km]')
plt.ylabel('[km]')
ax.set_title('Detumbling Orbit Dynamics')
plt.show()

# Plot Omega
f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
plt.rcParams['axes.grid'] = True
ax1.plot(time_hist/60,w_hist[:,0])
plt.ylabel('\omega_1')
plt.xlabel('t(min)')
plt.ylim([0, None])
ax1.set_title('Angular Velocity with Bdot controller')
ax2.plot(time_hist/60,w_hist[:,1])
plt.ylabel('\omega_2')
plt.xlabel('t(min)')
plt.ylim([0, None])
ax3.plot(time_hist/60,w_hist[:,2])
plt.ylabel('\omega_3')
plt.xlabel('t(min)')
plt.ylim([0, None])
plt.show()
#ps.runAll(inputs)
#orbit_sim.requestUplink()
#inputs2 = (None,None,None,None,None, None, None, None)
#ps.runAll(inputs2)