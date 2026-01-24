import wpilib
from phoenix6 import hardware, controls, configs
import constants


class Indexer:
    
    def __init__(self):
        # define all indexer moters and encoders/potentiometers and stuff
        self.bottomMoter = hardware.TalonFX(constants.INDEXER_BOTTOM_CAN_ID)
        self.topMoter = hardware.TalonFX(constants.INDEXER_TOP_CAN_ID)

        self.motorSpeed = 0
    
    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the indexer based off current state.
        """
        self.bottomMoter.set(self.motorSpeed)
        self.topMoter.set(-self.motorSpeed)

    def isManual(self):
        """
        Returns whether the indexer is being controlled by the operator
        
        Currently just returns True, but could return false when in auto
        """
        return True
