import magicbot
import phoenix6

from subsystem import shooter
from shooter.constants import HOOD_MAX_ANGLE, HOOD_MIN_ANGLE

class Hood:
    """Hood component

    This class controls the angle of the hood.
    """

    hood_motor: phoenix6.hardware.TalonFX
    hood_encoder: phoenix6.hardware.CANcoder

    def setup(self) -> None:
        """Set up initial state for the hood.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._hood_speed = 0.0
        self._hood_position = self.hood_encoder.get_position().value * 360.0

        self.is_manual = False  

        hood_configs = phoenix6.configs.TalonFXConfiguration()
        hood_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        hood_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.HOOD_ENCODER_CAN_ID
        )
        hood_configs.feedback.sensor_to_mechanism_ratio = (
            shooter.constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
        hood_configs.feedback.rotor_to_sensor_ratio = (
            shooter.constants.HOOD_ROTOR_TO_SENSOR_GEAR_RATIO
        )
        hood_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.CLOCKWISE_POSITIVE
        )
        # TODO: Configure soft limits.
        # TODO: Tune PID.
        self.hood_motor.configurator.apply(hood_configs)

        encoder_configs = phoenix6.configs.CANcoderConfiguration()
        encoder_configs.magnet_sensor.sensor_direction = (
            phoenix6.signals.spn_enums.SensorDirectionValue.CLOCKWISE_POSITIVE
        )
        self.hood_encoder.configurator.apply(encoder_configs)
        self._request = phoenix6.controls.PositionVoltage(self._hood_position)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        # TODO: Implement position control.
        # TODO: Implement velocity control (for homing).
        
        if self.is_manual:
            self.hood_motor.set(self._hood_speed)
        else:
            self.hood_motor.set_control(self._request.with_position(self._hood_position))

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._hood_speed = 0.0
        self._hood_position = self.hood_encoder.get_position().value * 360

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._hood_speed = 0.0
        self._hood_position = self.hood_encoder.get_position().value * 360

    def setSpeed(self, speed: float) -> None:
        """Set the speed of the hood."""
        self._hood_speed = speed

    def setPosition(self, position: float) -> None:
        """Set the position (in degrees) of the hood"""
        self._hood_position = max(HOOD_MIN_ANGLE, min(HOOD_MAX_ANGLE, position))

    def zeroEncoder(self) -> None:
        """Zeroes the encoder at its current position."""
        self.hood_encoder.set_position(0.0)

    @magicbot.feedback
    def get_hood_angle(self) -> float:
        return (
            self.hood_encoder.get_position().value
            * 360.0
            / shooter.constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
    