import wpilib
from phoenix6 import hardware, controls, configs
import constants


class Shooter:
    
    def __init__(self):
        # define all shooter moters and encoders/potentiometers and stuff
        self.turretMotor = hardware.TalonFX(constants.TURRET_CAN_ID)
        self.hoodMotor = hardware.TalonFX(constants.HOOD_CAN_ID)
        self.flywheelMotor = hardware.TalonFX(constants.FLYWHEEL_CAN_ID)

        turret_configs = configs.TalonFXConfiguration()
        turret_configs.feedback.sensor_to_mechanism_ratio = 41.67
        turret_configs.slot0.k_p = constants.SHOOTER_TURRET_P
        turret_configs.slot0.k_i = constants.SHOOTER_TURRET_I
        turret_configs.slot0.k_d = constants.SHOOTER_TURRET_D # TODO: tune these!
        self.turretMotor.configurator.apply(turret_configs)
        hood_configs = configs.TalonFXConfiguration()
        hood_configs.feedback.sensor_to_mechanism_ratio = 19
        hood_configs.slot0.k_p = constants.SHOOTER_HOOD_P
        hood_configs.slot0.k_i = constants.SHOOTER_HOOD_I
        hood_configs.slot0.k_d = constants.SHOOTER_HOOD_D
        self.hoodMotor.configurator.apply(hood_configs)

        self.turretAngle = 0
        self.hoodAngle = 0
        self.flywheelTargetVelocity = 0

        self.turretMotor.set_position(0)
        self.hoodMotor.set_position(0)
    
    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the shooter based off current state.
        """

        self.turretMotor.set_control(controls.PositionVoltage(self.turretAngle, constants.SHOOTER_TURRET_ROTATION_SPEED))
        self.hoodMotor.set_control(controls.PositionVoltage(self.hoodAngle, constants.SHOOTER_HOOD_ROTATION_SPEED))
        self.flywheelMotor.set(-self.flywheelTargetVelocity)

        # do any SmartDashboard stuff, if needed

    def isManual(self):
        """
        Returns whether the shooter is being controlled by the operator
        
        Currently just returns True, but will default to False when auto align is added,
        and could also return False if the autoalign doesnt work
        """
        return True