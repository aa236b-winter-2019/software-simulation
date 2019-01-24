from EarlyMissionOperations import EarlyMissionOperations
from ADCS import ADCS
from Standby import Standby
from PayloadOperations import PayloadOperations

if __name__ == '__main__':
    print('Beginning of Main File Running')

    early_mission_operations = EarlyMissionOperations()
    early_mission_operations.run()

    adcs = ADCS()
    adcs.run()

    standby = Standby()
    standby.run()

    payload_operations = PayloadOperations()
    payload_operations.run()
