

class HITLHardware(Hardware):
    """Class to represent the hardware in the Hardware in the Loop Simulation, running on the micro controller

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
