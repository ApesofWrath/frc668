import wpilib
from phoenix6 import hardware, controls, configs
import constants


class Hopper:
    
    def __init__(self):
        # define all hopper moters and encoders/potentiometers and stuff
        self.leftMoter = hardware.TalonFX(constants.HOPPER_LEFT_CAN_ID)
        self.rightMoter = hardware.TalonFX(constants.HOPPER_RIGHT_CAN_ID)

        self.motorSpeed = 0
    
    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the hopper based off current state.
        """
        self.leftMoter.set(self.motorSpeed)
        self.rightMoter.set(-self.motorSpeed)

    def isManual(self):
        """
        Returns whether the hopper is being controlled by the operator
        
        Currently just returns True, but could return false when in auto
        """
        return True
