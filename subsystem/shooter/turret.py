import magicbot
import phoenix6

from phoenix6.controls import PositionVoltage

from subsystem import shooter


class Turret:

    turret_motor: phoenix6.hardware.TalonFX
    turret_encoder: phoenix6.hardware.CANcoder

    def setup(self) -> None:
        """Set up initial state for the turret.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._turret_postion = 0.0

        turret_configs = phoenix6.configs.TalonFXConfiguration()
        turret_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.FUSED_CANCODER
        )
        turret_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.TURRET_ENCODER_CAN_ID
        )
        turret_configs.feedback.sensor_to_mechanism_ratio = (
            shooter.constants.TURRET_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
        turret_configs.feedback.rotor_to_sensor_ratio = (
            shooter.constants.TURRET_ROTOR_TO_SENSOR_GEAR_RATIO
        )
        turret_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.CLOCKWISE_POSITIVE
        )
        turret_configs.slot0.k_p = shooter.constants.TURRET_P
        turret_configs.slot0.k_i = shooter.constants.TURRET_I
        turret_configs.slot0.k_d = shooter.constants.TURRET_D
        self.turret_motor.configurator.apply(turret_configs)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.turret_motor.set_control(PositionVoltage(self._turret_postion))

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._turret_postion = 0.0

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._turret_postion = 0.0

    def setSpeed(self, speed: float) -> None:
        """Set the speed of the turret."""
        self._turret_postion = speed

    def zeroEncoder(self) -> None:
        """Zeroes the encoder at its current position."""
        self.turret_encoder.set_position(0.0)

    @magicbot.feedback
    def get_turret_angle(self) -> float:
        return (
            self.turret_encoder.get_position().value
            / shooter.constants.TURRET_SENSOR_TO_MECHANISM_GEAR_RATIO
        )