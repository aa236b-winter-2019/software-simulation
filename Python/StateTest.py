
# Based off state machine found here: https://python-3-patterns-idioms-test.readthedocs.io/en/latest/StateMachine.html

#TODO
# Make states return execution time
# Make run all return some form of what state the s/c is transitioning to
# Add power consumption to each state
# Add software in the loop hardware class


class State:
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"

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
        #super(Hold, Hold).propagate(hardware.current_state)

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
