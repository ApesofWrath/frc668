import wpilib
from phoenix6 import hardware, controls, configs
import constants


class Intake:
    
    def __init__(self):
        # define all intake motors and stuff TODO: there is annother motor for intake rotation
        self.intakeMotor = hardware.TalonFX(constants.INTAKE_CAN_ID)
        self.intakeRotatorMotor = hardware.TalonFX(constants.INTAKE_ROTATOR_CAN_ID)

        rotator_configs = configs.TalonFXConfiguration()
        rotator_configs.slot0.kP = constants.INTAKE_ROTATOR_P
        rotator_configs.slot0.kI = constants.INTAKE_ROTATOR_I
        rotator_configs.slot0.kD = constants.INTAKE_ROTATOR_D # TODO: tune these!
        self.intakeRotatorMotor.config_apply(rotator_configs)

        self.intakeSpeed = 0
        self.intakeAngle = 0


    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the intake based off current state.
        """
        self.intakeRotatorMotor.set_control(controls.PositionVoltage(self.intakeAngle))
        self.intakeMotor.set(self.intakeSpeed)



    def isManual(self):
        """
        Returns whether the intake is being controlled by the operator

        Currently just returns True, but could return false when in auto
        """
        return True