import wpilib
from phoenix6 import hardware, controls, configs, signals, StatusCode
import constants
from magicbot import feedback, tunable
from typing import Any


class Shooter:
    target_hood_angle = tunable(0.0)
    hood_kP = tunable(constants.SHOOTER_HOOD_P)
    hood_kD = tunable(constants.SHOOTER_HOOD_D)
    # If True, shooter mechanisms are controlled by joystick.
    joystick_control = tunable(True)
    logger: Any
    
    def __init__(self):
        # define all shooter moters and encoders/potentiometers and stuff
        self.turret_motor = hardware.TalonFX(constants.TURRET_CAN_ID)
        self.hood_motor = hardware.TalonFX(constants.HOOD_CAN_ID)
        self.fllywheel_motor = hardware.TalonFX(constants.FLYWHEEL_CAN_ID)

        self.hood_encoder = hardware.CANcoder(constants.HOOD_ENCODER_CAN_ID)

        turret_configs = configs.TalonFXConfiguration()
        # turret_configs.feedback.feedback_remote_sensor_id = constants.TURRET_ENCODER_CAN_ID
        turret_configs.feedback.sensor_to_mechanism_ratio = constants.TURRET_ENCODER_GEAR_RATIO # 41.67
        turret_configs.slot0.k_p = constants.SHOOTER_TURRET_P
        turret_configs.slot0.k_i = constants.SHOOTER_TURRET_I
        turret_configs.slot0.k_d = constants.SHOOTER_TURRET_D # TODO: tune these!
        self.turret_motor.configurator.apply(turret_configs)

        hood_configs = configs.TalonFXConfiguration()
        hood_configs.feedback.feedback_sensor_source = signals.spn_enums.FeedbackSensorSourceValue.REMOTE_CANCODER
        hood_configs.feedback.feedback_remote_sensor_id = constants.HOOD_ENCODER_CAN_ID
        hood_configs.feedback.sensor_to_mechanism_ratio = constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        hood_configs.feedback.rotor_to_sensor_ratio = constants.HOOD_ROTOR_TO_SENSOR_GEAR_RATIO
        hood_configs.slot0.k_p = constants.SHOOTER_HOOD_P
        hood_configs.slot0.k_i = constants.SHOOTER_HOOD_I
        hood_configs.slot0.k_d = constants.SHOOTER_HOOD_D
        self.hood_motor.configurator.apply(hood_configs)

        self.turret_rpm = 0
        self.hood_rpm = 0
        self.flywheel_target_rpm = 0

        self.turret_motor.set_position(0)
        self.hood_motor.set_position(0)
        self.hood_encoder.set_position(0)

        self.last_hood_kP = constants.SHOOTER_HOOD_P
        self.last_hood_kD = constants.SHOOTER_HOOD_D
    
    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the shooter based off current state.
        """
        if self.hood_kP != self.last_hood_kP or self.hood_kD != self.last_hood_kD:
            self.last_hood_kP = self.hood_kP
            self.last_hood_kD = self.hood_kD
            hood_configs = configs.TalonFXConfiguration()
            hood_configs.feedback.feedback_sensor_source = signals.spn_enums.FeedbackSensorSourceValue.REMOTE_CANCODER
            hood_configs.feedback.feedback_remote_sensor_id = constants.HOOD_ENCODER_CAN_ID
            hood_configs.feedback.sensor_to_mechanism_ratio = constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
            hood_configs.feedback.rotor_to_sensor_ratio = constants.HOOD_ROTOR_TO_SENSOR_GEAR_RATIO
            hood_configs.slot0.k_p = self.hood_kP
            hood_configs.slot0.k_i = constants.SHOOTER_HOOD_I
            hood_configs.slot0.k_d = self.hood_kD
            self.hood_motor.configurator.apply(hood_configs)

        if self.joystick_control:
            # Just use velocity control.
            self.hood_motor.set(-self.hood_rpm)
        else:
            # Command the hood motor to the target angle.
            if self.target_hood_angle < constants.HOOD_MIN_TARGET_ANGLE:
                self.logger.warning("Target hood angle {self.target_hood_angle} is out of range, clamping to {constants.HOOD_MIN_TARGET_ANGLE}")
                self.target_hood_angle = constants.HOOD_MIN_TARGET_ANGLE
            if self.target_hood_angle > constants.HOOD_MAX_TARGET_ANGLE:
                self.logger.warning("Target hood angle {self.target_hood_angle} is out of range, clamping to {constants.HOOD_MAX_TARGET_ANGLE}")
                self.target_hood_angle = constants.HOOD_MAX_TARGET_ANGLE
            status = self.hood_motor.set_control(controls.PositionVoltage(self.target_hood_angle))
            if not status.is_ok():
                self.logger.error(f"Could not set hood motor to target angle: {status}")

        # do any SmartDashboard stuff, if needed
    
    @feedback
    def get_hood_angle(self):
        return self.hood_encoder.get_position().value / constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO

    def isManual(self):
        """
        Returns whether the shooter is being controlled by the operator
        
        Currently just returns True, but will default to False when auto align is added,
        and could also return False if the autoalign doesnt work
        """
        return True