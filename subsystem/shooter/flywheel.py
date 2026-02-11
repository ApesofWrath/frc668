import magicbot
import phoenix6

from subsystem import shooter


class Flywheel:
    """Flywheel component

    This class controls the rotational velocity of the flywheel.
    """

    flywheel_motor: phoenix6.hardware.TalonFX
    flywheel_encoder: phoenix6.hardware.CANcoder

    def setup(self) -> None:
        """Set up initial state for the flywheel.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # Configure the feedback sensor to use and the feedforward + feedback
        # gains.
        self._flywheel_configs = phoenix6.configs.TalonFXConfiguration()
        self._flywheel_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        self._flywheel_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.FLYWHEEL_ENCODER_CAN_ID
        )
        self._flywheel_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.CLOCKWISE_POSITIVE
        )
        self._flywheel_configs.motor_output.neutral_mode = (
            phoenix6.signals.spn_enums.NeutralModeValue.COAST
        )
        # Output to overcome static friction
        self._flywheel_configs.slot0.k_s = shooter.constants.FLYWHEEL_K_S
        # A target of 1 rps results in this output
        self._flywheel_configs.slot0.k_v = shooter.constants.FLYWHEEL_K_V
        # An acceleration of 1 rps/s requires this output
        self._flywheel_configs.slot0.k_a = shooter.constants.FLYWHEEL_K_A
        # An error of 1 rps results in this output
        self._flywheel_configs.slot0.k_p = shooter.constants.FLYWHEEL_K_P
        # Accumulated error of 1 rps results in this output
        self._flywheel_configs.slot0.k_i = shooter.constants.FLYWHEEL_K_I
        # A rate of change of error of 1 rps/s results in this output
        self._flywheel_configs.slot0.k_d = shooter.constants.FLYWHEEL_K_D
        self.flywheel_motor.configurator.apply(self._flywheel_configs)

        self._flywheel_encoder_configs = (
            phoenix6.configs.CANcoderConfiguration()
        )
        self._flywheel_encoder_configs.magnet_sensor.sensor_direction = (
            phoenix6.signals.spn_enums.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.flywheel_encoder.configurator.apply(self._flywheel_encoder_configs)

        self._target_rps: float = 0.0

        # Create a velocity closed-loop request with voltage output and slot 0
        # configs.
        self._request = phoenix6.controls.VelocityVoltage(
            self._target_rps
        ).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.flywheel_motor.set_control(
            self._request.with_velocity(self._target_rps)
        )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._target_rps = shooter.constants.FLYWHEEL_ENABLE_RPS

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._target_rps = 0.0

    def setTargetRps(self, target_rps: float) -> None:
        self._target_rps = target_rps

    @magicbot.feedback
    def get_target_rps(self) -> float:
        return self._target_rps


class FlywheelTuner:
    """Component for tuning the flywheel gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable flywheel target velocity.
    """

    flywheel_motor: phoenix6.hardware.TalonFX
    flywheel_encoder: phoenix6.hardware.CANcoder
    flywheel: Flywheel

    # Gains for velocity control of the flywheel.
    k_s = magicbot.tunable(shooter.constants.FLYWHEEL_K_S)
    k_v = magicbot.tunable(shooter.constants.FLYWHEEL_K_V)
    k_a = magicbot.tunable(shooter.constants.FLYWHEEL_K_A)
    k_p = magicbot.tunable(shooter.constants.FLYWHEEL_K_P)
    k_i = magicbot.tunable(shooter.constants.FLYWHEEL_K_I)
    k_d = magicbot.tunable(shooter.constants.FLYWHEEL_K_D)

    # The target rotational velocity of the flywheel.
    target_rps = magicbot.tunable(shooter.constants.FLYWHEEL_ENABLE_RPS)

    def setup(self) -> None:
        """Set up initial state for the flywheel tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self.last_k_s = shooter.constants.FLYWHEEL_K_S
        self.last_k_v = shooter.constants.FLYWHEEL_K_V
        self.last_k_a = shooter.constants.FLYWHEEL_K_A
        self.last_k_p = shooter.constants.FLYWHEEL_K_P
        self.last_k_i = shooter.constants.FLYWHEEL_K_I
        self.last_k_d = shooter.constants.FLYWHEEL_K_D

    def execute(self) -> None:
        """Update the flywheel speed and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.flywheel.setTargetRps(self.target_rps)

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
        result = self.flywheel_motor.configurator.apply(slot0_configs)
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to flywheel motor")

    @magicbot.feedback
    def get_motor_voltage(self) -> phoenix6.units.volt:
        return self.flywheel_motor.get_motor_voltage().value

    @magicbot.feedback
    def get_motor_supply_current(self) -> phoenix6.units.ampere:
        return self.flywheel_motor.get_supply_current().value

    @magicbot.feedback
    def get_motor_stator_current(self) -> phoenix6.units.ampere:
        return self.flywheel_motor.get_stator_current().value

    @magicbot.feedback
    def get_measured_rps(self) -> float:
        return self.flywheel_encoder.get_velocity().value
