class Environment:
    def propagate(self):
        assert 0, "propagate not implemented"
        
class Simulation(Environment):
    def __init__(self):
        self.state = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] 
        # The state vector [jx, jy, jz, then 4 quaternions, x, y, z, xdot, ydot, zdot]
        self.uplinkRequested = False
        self.downlinkRequested = False
    def propagate(self, initial_state, control_effort, timestep):
        # This should call orbit propagator
        print('Propagating the time forward by %.1f seconds' %timestep)

        return initial_state + 1*timestep
        
        
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
        
        
    def getBatteryVoltage(self):
        return self.battery_voltage
    
    def getAngularVelocity(self):
        # How are we going to estimate how the vechile perceives it's actual state?
        return self.angular_velocity


# Based off state machine found here: https://python-3-patterns-idioms-test.readthedocs.io/en/latest/StateMachine.html

class State:
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"

    @staticmethod
    def propagate(hardware, initial_state, control_effort, timestep):
        return hardware.environment.propagate(initial_state, control_effort, timestep)

class StateMachine:
    def __init__(self, initialState, hardware):
        self.currentState = initialState
        self.hardware = hardware
        self.currentState.run(self.hardware)
        
    # Template method:
    def runAll(self, inputs):
        for i in inputs:
            #print(i)
            self.currentState = self.currentState.next(self.hardware)
            self.currentState.run(self.hardware)
            
    def runNext(self, inputs):
        self.currentState = self.currentState.next(inputs, self.hardware)
        self.currentState.run(self.hardware)





class Hold(State):
    def run(self, hardware):
        print("Hold: Initial Boot and hold")
        super(Hold, Hold).propagate(hardware.current_state)

    def next(self, hardware):
        return PandaSat.deploy_antenna
    
    def __str__(self):
        return('Hold state')

class DeployAntenna(State):
    def run(self, hardware):
        print("Deploying Antenna")

    def next(self, hardware):
        return PandaSat.tumble_check
    
    def __str__(self):
        return('Deploy antenna state')
    
class TumbleCheck(State):
    def run(self, hardware):
        print('Checking the tumbling rate')
        
    def next(self, hardware):
        #tumbling
        if self.tumble_compare(hardware):
            return PandaSat.battery_tumble_check()
        return PandaSat.sleep
    
    def __str__(self):
        return('Tumble check state')
        
    def tumble_compare(self, hardware):
        # Implements the logic of the tumble check
        # Return True for tumbling too fast or False for not tumbling above the threshold
        threshold_value = 1 #1 rad/s?
        if any(item > threshold_value for item in hardware.getAngularVelocity()):
            return True
        return False
        
class Sleep(State):
    def run(self, hardware):
        print('Sleep Mode - 1 min')
        
    def next(self, hardware):
        return PandaSat.battery_beacon_check
    
    def __str__(self):
        return('Sleep state')
    
class BatteryTumbleCheck(State):
    def run(self, hardware):
        print('Checking battery voltage before 1 min of detumbling')
        
    def next(self, hardware):
        battery_voltage = hardware.getBatteryVoltage()
        battery_tumble_threshold = hardware.battery_tumble_threshold
        if battery_voltage < battery_tumble_threshold:
            return PandaSat.sleep
        return PandaSat.detumble
    
    def __str__(self):
        return('Battery tumble check state')
    
class Detumble(State):
    def run(self, hardware):
        print('Detumbling the spacecraft for 1 min')
        
    def next(self, hardware):
        return PandaSat.battery_beacon_check
    
    def __str__(self):
        return('Detumble state')
    
class BatteryBeaconCheck(State):
    def run(self, hardware):
        print('Checking battery voltage before sending beacon')
        
    def next(self, hardware):
        battery_voltage = hardware.getBatteryVoltage()
        battery_beacon_threshold = hardware.battery_beacon_threshold
        if battery_voltage < battery_beacon_threshold:
            return PandaSat.sleep
        return PandaSat.beacon
    
    def __str__(self):
        return('Battery beacon check state')
    
class Beacon(State):
    def run(self, hardware):
        print('Sending health data beacon signal')
    
    def next(self, hardware):
        return PandaSat.listen
    
    def __str__(self):
        return('Beacon state')

class Listen(State):
    def run(self, hardware):
        print('Listening for ground signal for ~15s')
        
    def next(self, hardware):
        return PandaSat.uplink_check
    
    def __str__(self):
        return('Listen state')
    
class UplinkCheck(State):
    def run(self, hardware):
        print('Checking if an uplink was made')
        
    def next(self, hardware):
        if hardware.environment.uplinkRequested:
            return PandaSat.process_uplink
        return PandaSat.payload_schedule_check
        
    def __str__(self):
        return('Uplink check state')
    
class ProcessUplink(State):
    def run(self, hardware):
        print('Processing uplink')
        hardware.environment.uplinkRequested = False #Resets this flag
        
    def next(self, hardware):
        return PandaSat.downlink_check
    
    def __str__(self):
        return('Process uplink state')
    
class DownlinkCheck(State):
    def run(self, hardware):
        print('Checking for downlink')
        
    def next(self, hardware):
        if hardware.environment.downlinkRequested:
            return PandaSat.downlink
        return PandaSat.payload_schedule_check
    def __str__(self):
        return('Downlink check state')
    
class Downlink(State):
    def run(self, hardware):
        print('Downlinking to ground station')
        
    def next(self, hardware):
        return PandaSat.payload_schedule_check
    
    def __str__(self):
        return('Downlink state')
        
class PayloadScheduleCheck(State):
    def run(self, hardware):
        print('Checking if a payload operation is requested')
        
    def next(self, hardware):
        current_time = hardware.time
        margin= 60 #seconds but should convert to julian date
        
        if current_time > hardware.payload_time[0] - margin:
            return PandaSat.battery_payload_check
        return PandaSat.sleep
    
class BatteryPayloadCheck(State):
    def run(self, hardware):
        print('Checking battery voltage before turning payload on')
        
    def next(self, hardware):
        battery_voltage = hardware.getBatteryVoltage()
        battery_payload_threshold = hardware.battery_payload_threshold
        if battery_voltage < battery_beacon_threshold:
            return PandaSat.sleep
        return PandaSat.payload_on
    
    def __str__(self):
        return('Battery payload check state')
    
class PayloadOn(State):
    def run(self, hardware):
        print('Turn the payload on and run')
        
    def next(self, hardware):
        return PandaSat.sleep
        

class PandaSat(StateMachine):
    def __init__(self, hardware):
        # Initial state
        StateMachine.__init__(self, PandaSat.hold, hardware)

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





orbit_sim = Simulation()
hardware = PhysicalSat(orbit_sim)
inputs = (None,None,None,None,None, None, None, None, None)
ps = PandaSat(hardware)
ps.runAll(inputs)
print('\n')
orbit_sim.requestUplink()
inputs2 = (None,None,None,None,None, None, None, None)
ps.runAll(inputs2)