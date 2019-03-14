import numpy as np
from StateMachine import *
from igrffx import igrffx
from dynamics import dynamics
import julian
from propagate import propagate
from subroutines import *
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from CPsubroutines import *

# TODO
# Put actual numbers in for the power dictionary
# Fix runTime method

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
        self.m_max = np.array([[.0096, .0096, .0096]])
        self.power_max = voltage_max*I_max                  # Max power consumed (W)
        self.m_value = [[0, 0, 0]] #maqnetorquer initially off

        self.initializeState()
        self.setPowerDraw()
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
        B_eci = igrffx(rv_eci[0:3],t0) 
        

        std_om = 8.75*10**-3             # std dev in deg/s
        std_b = 0.14*10**-3              # std dev in guass

        om = omq[:3]*180.0/np.pi + np.random.normal(0,std_om) # converting to deg/s and adding noise
        B_eci = B_eci*10**4 +np.random.normal(0,std_b)             # converting to gauss and adding noise

        q = omq[3:7]

        # Rotate B_eci into Satellite Body Frame
        B_body = np.dot(q2rot(q),B_eci)   

        writeToSerial([[0,0,0], om, B_body, 0, 0], 5)

        return ([0, 0, 0], om, B_body)


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

    def initializeState(self):
        # Orbital Elements and Parameters
        a = 6917.50              # Orbit Semi-Major Axis (km)
        e = 0.000287             # Eccentricity
        i = 28.47                # Inclination (deg)
        RAAN = 176.23            # Right Ascension of Ascending Node (deg)
        w = 82.61                # Argument of Perigee (deg)
        anom = 100            # Mean Anomaly (deg)
        mu = 3.986e5             # Earth Standard Gravitational Parameter (km^3/s^2)
        T = 2*np.pi*np.sqrt((a**3)/mu) # Orbital Period (s)
        r_eci, v_eci = OE2ECI(a, e, i, RAAN, w, anom, mu)

        # Initialize Known Variables and Initial Conditions
        rv_eci0 = np.append(r_eci, v_eci)   # Initial Orbit State Vector
        om = 0.25*np.array([1, 1, 1])         # Initial Angular Velocity (rad/s)
        q = np.array([0,0,1,0])             # Initial Quaternion, Identity
        torque = np.array([0,0,0])          # Initial Torque
        power = np.array([0, 0])             # Initial Power consumption and generation
        omqtp0 = np.concatenate((om,q)) # Initial Attitude State Vector  

        self.state = np.concatenate((rv_eci0, omqtp0))

    def setPowerDraw(self):
        self.power_draw_dict = {'Hold state': 1.0, 
            'Deploy antenna state': 1.1,
            'Tumble check state': 1.2,
            'Sleep state': 1.3,
            'Battery tumble check state': 1.4,
            'Detumble state': 1.5, 
            'Battery beacon check state': 1.6,
            'Beacon state': 1.7,
            'Listen state': 1.8,
            'Uplink check state': 1.9,
            'Process uplink state': 2.0,
            'Downlink check state': 2.1,
            'Downlink state': 2.2,
            'Payload schedule check state': 2.3,
            'Battery payload check state': 2.4,
            'Payload on state': 2.5}


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

    xyz_hist = np.array([None, None, None])
    w_hist = []
    q_hist = []
    torque_hist = []
    power_hist = []
    time_hist = []

    #J, J_inv, m_max, power_max = ps.hardware.getHardwareProperties()
    time = 0
    while time <= max_time:
        delta_t, _ = state_machine.runStep(None)
        state_machine.hardware.state = propagate(state_machine.hardware, delta_t, str(state_machine.getCurrentState()))
        state_machine.hardware.time += (delta_t/86400)
        time += delta_t
        state_machine.hardware.time += (delta_t/86400)
        print('current time: %.1f min' %(time/60))

        xyz_hist = np.vstack((xyz_hist, state_machine.hardware.state[0:3]))
        w_hist = np.append(w_hist, state_machine.hardware.state[6:9])
        q_hist = np.append(q_hist, state_machine.hardware.state[9:13])
        torque_hist = np.append(torque_hist, state_machine.hardware.state[13:16])
        power_hist = np.append(power_hist, state_machine.hardware.state[16:18])
        time_hist = np.append(time_hist, time)

    return (time_hist, xyz_hist, w_hist, q_hist, torque_hist, power_hist)

def runSimulationSteps(state_machine, num_steps):
    # Assumptions
    # time starts at zero
    xyz_hist = np.zeros((num_steps, 3))
    w_hist = np.zeros((num_steps, 3))
    q_hist = np.zeros((num_steps, 4))
    torque_hist = np.zeros((num_steps, 3))
    power_hist = np.zeros((num_steps, 2))
    time_hist = np.zeros(num_steps)


    time = 0
    for i in range(num_steps):
        #print(state_machine.getCurrentState())
        delta_t, _ = state_machine.runStep(None)
        state_machine.hardware.state = propagate(state_machine.hardware, delta_t, str(state_machine.getCurrentState()))
        state_machine.hardware.time += (delta_t/86400)
        time += delta_t
        xyz_hist[i, :] = state_machine.hardware.state[0:3]
        w_hist[i,:] = state_machine.hardware.state[6:9]
        q_hist[i,:] = state_machine.hardware.state[9:13]
        torque_hist[i,:] = state_machine.hardware.state[13:16]
        power_hist[i,:] = state_machine.hardware.state[16:18]
        time_hist[i] = time
        #print('current time: %.1f min' %(time/60))

    return (time_hist, xyz_hist, w_hist, q_hist, torque_hist, power_hist)



def plotValues(time_hist, xyz_hist, w_hist, q_hist, torque_hist, power_hist):

    # Plot Orbit 
    plt.figure()
    ax=plt.axes(projection='3d')
    ax.plot3D(xyz_hist[:,0],xyz_hist[:,1],xyz_hist[:,2],'r')
    ax.axis('equal')
    plt.xlabel('[km]')
    plt.ylabel('[km]')
    ax.set_title('Detumbling Orbit Dynamics')

    # Plot Omega
    f,(ax1,ax2,ax3)=plt.subplots(3,1, sharey=True)
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

    # Plot Quaternion
    f,(ax1,ax2,ax3,ax4)=plt.subplots(4,1,sharey=True)
    plt.rcParams['axes.grid'] = True
    ax1.plot(time_hist/60,q_hist[:,0])
    plt.ylabel('q_1')
    plt.xlabel('t(min)')
    ax1.set_title('Quaternion Dynamics')
    ax2.plot(time_hist/60,q_hist[:,1])
    plt.ylabel('q_2')
    plt.xlabel('t(sec)')
    ax3.plot(time_hist/60,q_hist[:,2])
    plt.ylabel('q_3')
    plt.xlabel('t(min)')
    ax4.plot(time_hist/60,q_hist[:,3])
    plt.ylabel('q_4')
    plt.xlabel('t(min)')

    '''
    # Plot Torque
    #Tspan_torque = np.arange(0, T/n*epochs, T/n)
    f,(ax1,ax2,ax3)=plt.subplots(3,1,sharey=True)
    plt.rcParams['axes.grid'] = True
    ax1.plot(time_hist/60,torque_hist[:,0])
    plt.ylabel('Tx()')
    plt.xlabel('t(min)')
    ax1.set_title('Torque')
    ax2.plot(time_hist/60,torque_hist[:,1])
    plt.ylabel('Ty()')
    plt.xlabel('t(min)')
    ax3.plot(time_hist/60,torque_hist[:,2])
    plt.ylabel('Tz()')
    plt.xlabel('t(min)')

    # Plot Power and Compute Energy Consumption
    energy_consumed = np.trapz(power_hist[:,0],time_hist)
    print('Total energy consumed ' + str(energy_consumed))
    plt.figure()
    plt.plot(time_hist/60, power_hist[:,0],'r')
    plt.xlabel('Time (min)')
    plt.ylabel('Power consumption (W)')
    plt.title('Power consumption ')
    '''
    plt.show()

#Simulation


#orbit_sim = SoftwareSimulation()
hardware = SoftwareSimHardware()
#inputs = (None,None,None,None,None, None, None, None, None)
ps = PandaSat(hardware)
time_hist, xyz_hist, w_hist, q_hist, torque_hist, power_hist = runSimulationSteps(ps, 5)
#print(w_hist)

plotValues(time_hist, xyz_hist, w_hist, q_hist, torque_hist, power_hist)

#ps.runAll(inputs)
#orbit_sim.requestUplink()
#inputs2 = (None,None,None,None,None, None, None, None)
#ps.runAll(inputs2)