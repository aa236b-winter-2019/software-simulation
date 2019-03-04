
# Based off state machine found here: https://python-3-patterns-idioms-test.readthedocs.io/en/latest/StateMachine.html

#TODO
# Make Hold state only run 1 min at a time for B_eci's sake
# 
# REMOVE NUMPY FROM DETUMBLE CALCS

import math
import numpy as np

class State:
    verbose_flag = False
    def run(self):
        assert 0, "run not implemented"
    def next(self, input):
        assert 0, "next not implemented"

class StateMachine:
    def __init__(self, initialState, hardware):
        self.currentState = initialState
        self.hardware = hardware
        #self.currentState.run(self.hardware)
        
    # Template method:
    def runAll(self, inputs):
        for i in inputs:
            #print(i)
            self.currentState.run(self.hardware)
            self.currentState = self.currentState.next(self.hardware)
            
            
    def runStep(self, inputs):
        delta_t = self.currentState.run(self.hardware)
        next_state = self.currentState.next(self.hardware)
        self.currentState = next_state

        return (delta_t, str(next_state))

    def getCurrentState(self):
        return self.currentState


class Hold(State):
    def run(self, hardware):
        TIME = 60 # seconds
        if State.verbose_flag:
            print("Hold: Initial Boot and hold")
        #super(Hold, Hold).propagate(hardware.current_state)
        return TIME

    def next(self, hardware):
        return PandaSat.deploy_antenna
    
    def __str__(self):
        return('Hold state')

class DeployAntenna(State):
    def run(self, hardware):
        TIME = 60 # seconds
        if State.verbose_flag:
            print("Deploying Antenna")
        return TIME

    def next(self, hardware):
        return PandaSat.tumble_check
    
    def __str__(self):
        return('Deploy antenna state')
    
class TumbleCheck(State):
    #FIX THE LOGIC HERE!!!!
    
    tumble_threshold_value = .0175 #rad/s = 1 deg/s
    def run(self, hardware):
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking the tumbling rate')
        return TIME
        
    def next(self, hardware):
        #tumbling
        if self.tumble_compare(hardware):
            return PandaSat.battery_tumble_check
        return PandaSat.sleep
    
    def __str__(self):
        return('Tumble check state')
        
    def tumble_compare(self, hardware):
        # Implements the logic of the tumble check
        # Return True for tumbling too fast or False for not tumbling above the threshold
        spin_rate = hardware.readIMU()[1]*(3.14/180.0)   # reads IMU measurement in deg/s and converts to rad/s

        spin_magnitude = math.sqrt(spin_rate[0]**2 + spin_rate[1]**2 + spin_rate[2]**2)
        if spin_magnitude > TumbleCheck.tumble_threshold_value:
            return True
        return False
        
class Sleep(State):
    def run(self, hardware):
        TIME = 60 # seconds
        if State.verbose_flag:
            print('Sleep Mode - 1 min')
        return TIME
        
    def next(self, hardware):
        return PandaSat.battery_beacon_check
    
    def __str__(self):
        return('Sleep state')
    
class BatteryTumbleCheck(State):
    #CHANGE TO CHECK BATTERY VOLTAGE ON ACTUAL FLIGHT SOFTWARE
    def run(self, hardware):
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking battery voltage before 1 min of detumbling')
        return TIME
        
    def next(self, hardware):
        battery_voltage = hardware.checkBatteryPercent()
        battery_tumble_threshold = 30 #Percentage
        if battery_voltage < battery_tumble_threshold:
            return PandaSat.sleep
        return PandaSat.detumble
    
    def __str__(self):
        return('Battery tumble check state')
    
class Detumble(State):
    detumble_second_count = 0
    def run(self, hardware):
        Detumble.detumble_second_count += 1
        TIME = 1 # length of time 
        m_value = self.calcMValue(hardware)
        hardware.runMagnetorquer( m_value)
        if State.verbose_flag:
            print('Detumbling the spacecraft for 1 s')
        return TIME
        
    def next(self, hardware):
        #ANY NEXT STATE SHOULD CALL hardware.runMagnetorquer([0,0,0])
        # EXCEPT DETUMBLE
        if Detumble.detumble_second_count >= 60:
            Detumble.detumble_second_count = 0
            #hardware.runMagnetorquer([0,0,0]) #Turns the magnetorquer off
            return PandaSat.battery_beacon_check
        return PandaSat.detumble

    def calcMValue(self, hardware):
        imu_reading = hardware.readIMU()

        om0 = imu_reading[1]*(3.14/180.0)
        B_body = imu_reading[2]*10**-4

        B_dot = -np.cross(om0,B_body)               # Compute B_dot
        m_value = -np.multiply(hardware.m_max,np.sign(B_dot))*np.linalg.norm(np.tanh(om0))

        return m_value

    
    def __str__(self):
        return('Detumble state')
    
class BatteryBeaconCheck(State):
    battery_beacon_threshold = 30 #Percent
    def run(self, hardware):
        hardware.runMagnetorquer([[0,0,0]])
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking battery voltage before sending beacon')
        return TIME
        
    def next(self, hardware):
        battery_voltage = hardware.checkBatteryPercent()
        if battery_voltage < BatteryBeaconCheck.battery_beacon_threshold:
            return PandaSat.sleep
        return PandaSat.beacon
    
    def __str__(self):
        return('Battery beacon check state')
    
class Beacon(State):
    def run(self, hardware):
        TIME = 10 # seconds
        if State.verbose_flag:
            print('Sending health data beacon signal')
        return TIME
    
    def next(self, hardware):
        return PandaSat.listen
    
    def __str__(self):
        return('Beacon state')

class Listen(State):
    def run(self, hardware):
        TIME = 15 # seconds
        if State.verbose_flag:
            print('Listening for ground signal for 15s')
        return TIME
        
    def next(self, hardware):
        return PandaSat.uplink_check
    
    def __str__(self):
        return('Listen state')
    
class UplinkCheck(State):
    def run(self, hardware):
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking if an uplink was made')
        return TIME
        
    def next(self, hardware):
        if hardware.uplink_requested:
            return PandaSat.process_uplink
        return PandaSat.payload_schedule_check
        
    def __str__(self):
        return('Uplink check state')
    
class ProcessUplink(State):
    def run(self, hardware):
        TIME = 10 # seconds
        if State.verbose_flag:
            print('Processing uplink')
        hardware.environment.uplinkRequested = False #Resets this flag
        return TIME
        
    def next(self, hardware):
        return PandaSat.downlink_check
    
    def __str__(self):
        return('Process uplink state')
    
class DownlinkCheck(State):
    def run(self, hardware):
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking for downlink')
        return TIME
        
    def next(self, hardware):
        if hardware.downlinkRequested:
            return PandaSat.downlink
        return PandaSat.payload_schedule_check
    def __str__(self):
        return('Downlink check state')
    
class Downlink(State):
    def run(self, hardware):
        TIME = 5*60 # seconds
        if State.verbose_flag:
            print('Downlinking to ground station')
        return TIME
        
    def next(self, hardware):
        return PandaSat.payload_schedule_check
    
    def __str__(self):
        return('Downlink state')
        
class PayloadScheduleCheck(State):
    def run(self, hardware):
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking if a payload operation is requested')
        return TIME
        
    def next(self, hardware):
        current_time = hardware.time
        margin = 60 #seconds but should convert to julian date
        
        if current_time > hardware.payload_time[0] - margin:
            return PandaSat.battery_payload_check
        return PandaSat.tumble_check

    def __str__(self):
        return('Payload schedule check state')
    
class BatteryPayloadCheck(State):
    battery_payload_threshold = 30 # Percent
    def run(self, hardware):
        TIME = 1 # seconds
        if State.verbose_flag:
            print('Checking battery voltage before turning payload on')
        return TIME
        
    def next(self, hardware):
        battery_voltage = hardware.checkBatteryPercent()
        if battery_voltage < BatteryPayloadCheck.battery_beacon_threshold:
            return PandaSat.tumble_check
        return PandaSat.payload_on
    
    def __str__(self):
        return('Battery payload check state')
    
class PayloadOn(State):
    def run(self, hardware):
        TIME = 5*60 # seconds
        if State.verbose_flag:
            print('Turn the payload on and run')
        return TIME
        
    def next(self, hardware):

        return PandaSat.tumble_check

    def __str__(self):
        return('Payload on state')
        

class PandaSat(StateMachine):
    def __init__(self, hardware):
        # Initial state
        StateMachine.__init__(self, PandaSat.detumble, hardware)


