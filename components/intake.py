import wpilib
from phoenix6 import hardware, controls, configs
import constants


class Intake:
    
    def __init__(self):
        # define all intake motors and stuff TODO: there is annother motor for intake rotation
        self.intakeMotor = hardware.TalonFX(constants.INTAKE_CAN_ID)
        self.intakeSpeed = 0


    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the intake based off current state.
        """
        self.intakeMotor.set(self.intakeSpeed)



    def isManual(self):
        """
        Returns whether the intake is being controlled by the operator

        Currently just returns True, but could return false when in auto
        """
        return True