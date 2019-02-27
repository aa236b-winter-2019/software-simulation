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
    time = 0
    while time <= max_time:
        delta_t, _ = state_machine.runStep(None)
        time += delta_t
        print('current time: %.1f min' %(time/60))

def runSimulationSteps(state_machine, num_steps):
    # Assumptions
    # time starts at zero
    time = 0
    for i in range(num_steps):
        delta_t, _ = state_machine.runStep(None)
        time += delta_t
        print('current time: %.1f min' %(time/60))


#Simulation


orbit_sim = SoftwareSimulation()
hardware = PhysicalSat(orbit_sim)
inputs = (None,None,None,None,None, None, None, None, None)
ps = PandaSat(hardware)
runSimulationSteps(ps, 10)
#ps.runAll(inputs)
print('\n')
#orbit_sim.requestUplink()
#inputs2 = (None,None,None,None,None, None, None, None)
#ps.runAll(inputs2)