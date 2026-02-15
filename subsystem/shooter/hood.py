import magicbot
import phoenix6

from subsystem import shooter
from subsystem.shooter.constants import HOOD_MAX_ANGLE, HOOD_MIN_ANGLE


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

        hood_configs.slot0.k_s = shooter.constants.HOOD_K_S
        hood_configs.slot0.k_v = shooter.constants.HOOD_K_V
        hood_configs.slot0.k_a = shooter.constants.HOOD_K_A
        hood_configs.slot0.k_p = shooter.constants.HOOD_K_P
        hood_configs.slot0.k_i = shooter.constants.HOOD_K_I
        hood_configs.slot0.k_d = shooter.constants.HOOD_K_D

        # TODO: Configure soft limits.
        self.hood_motor.configurator.apply(hood_configs)

        encoder_configs = phoenix6.configs.CANcoderConfiguration()
        encoder_configs.magnet_sensor.sensor_direction = (
            phoenix6.signals.spn_enums.SensorDirectionValue.CLOCKWISE_POSITIVE
        )
        self.hood_encoder.configurator.apply(encoder_configs)

        self._request = phoenix6.controls.PositionVoltage(
            self._hood_position
        ).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        # TODO: Implement position control.
        # TODO: Implement velocity control (for homing).

        if self.is_manual:
            self.hood_motor.set(self._hood_speed)
        else:
            self.hood_motor.set_control(
                self._request.with_position(self._hood_position / 360)
            )

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

    def setPosition(self, target_position: float) -> None:
        """Set the position (in degrees) of the hood"""
        self._hood_position = target_position
        # max(
        #     HOOD_MIN_ANGLE, min(HOOD_MAX_ANGLE, target_position)
        # )

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


class HoodTuner:
    """Component for tuning the hood gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable hood target angle position.
    """

    hood_motor: phoenix6.hardware.TalonFX
    hood_encoder: phoenix6.hardware.CANcoder
    hood: Hood

    # Gains for position control of the hood.
    k_s = magicbot.tunable(shooter.constants.HOOD_K_S)
    k_v = magicbot.tunable(shooter.constants.HOOD_K_V)
    k_a = magicbot.tunable(shooter.constants.HOOD_K_A)
    k_p = magicbot.tunable(shooter.constants.HOOD_K_P)
    k_i = magicbot.tunable(shooter.constants.HOOD_K_I)
    k_d = magicbot.tunable(shooter.constants.HOOD_K_D)

    # The target angle of the hood, in degrees.
    target_angle = magicbot.tunable(0.0)

    def setup(self) -> None:
        """Set up initial state for the hood tuner.
        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self.last_k_s = shooter.constants.HOOD_K_S
        self.last_k_v = shooter.constants.HOOD_K_V
        self.last_k_a = shooter.constants.HOOD_K_A
        self.last_k_p = shooter.constants.HOOD_K_P
        self.last_k_i = shooter.constants.HOOD_K_I
        self.last_k_d = shooter.constants.HOOD_K_D

    def execute(self) -> None:
        """Update the hood speed and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.hood.setPosition(self.target_angle)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if not self.gainsChanged():
            return

        self.applyGains()

        self.last_k_s = self.k_s
        self.last_k_v = self.k_v
        self.last_k_a = self.k_a
        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.k_s != self.last_k_s
            or self.k_v != self.last_k_v
            or self.k_a != self.last_k_a
            or self.k_p != self.last_k_p
            or self.k_i != self.last_k_i
            or self.k_d != self.last_k_d
        )

    def applyGains(self) -> None:
        """Apply the current gains to the motor."""
        slot0_configs = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.k_s)
            .with_k_v(self.k_v)
            .with_k_a(self.k_a)
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )
        result = self.hood_motor.configurator.apply(slot0_configs)
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to hood motor")

    @magicbot.feedback
    def get_motor_voltage(self) -> phoenix6.units.volt:
        return self.hood_motor.get_motor_voltage().value

    @magicbot.feedback
    def get_motor_supply_current(self) -> phoenix6.units.ampere:
        return self.hood_motor.get_supply_current().value

    @magicbot.feedback
    def get_motor_stator_current(self) -> phoenix6.units.ampere:
        return self.hood_motor.get_stator_current().value

    @magicbot.feedback
    def get_hood_angle(self) -> float:
        return (
            self.hood_encoder.get_position().value
            * 360.0
            / shooter.constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        )

    @magicbot.feedback
    def get_encoder_position(self) -> float:
        return self.hood_encoder.get_position().value
