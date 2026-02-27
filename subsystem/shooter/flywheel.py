import enum

import magicbot
import phoenix6

from subsystem import shooter


class FlywheelControlType(enum.Enum):
    VOLTAGE = 0
    TORQUE_CURRENT_FOC = 1


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
        # Configure the motor.
        # https://api.ctr-electronics.com/phoenix6/stable/python/autoapi/phoenix6/configs/talon_fx_configs/index.html
        self.flywheel_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration()
            .with_feedback(
                phoenix6.configs.FeedbackConfigs()
                .with_feedback_sensor_source(
                    phoenix6.signals.FeedbackSensorSourceValue.FUSED_CANCODER
                )
                .with_feedback_remote_sensor_id(
                    shooter.constants.FLYWHEEL_ENCODER_CAN_ID
                )
            )
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(
                    phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
                )
                .with_neutral_mode(phoenix6.signals.NeutralModeValue.COAST)
            )
            .with_slot0(
                # Use slot 0 for voltage control gains.
                phoenix6.configs.Slot0Configs()
                .with_k_p(shooter.constants.FLYWHEEL_VOLTAGE_K_P)
                .with_k_i(shooter.constants.FLYWHEEL_VOLTAGE_K_I)
                .with_k_d(shooter.constants.FLYWHEEL_VOLTAGE_K_D)
                .with_k_s(shooter.constants.FLYWHEEL_VOLTAGE_K_S)
                .with_k_v(shooter.constants.FLYWHEEL_VOLTAGE_K_V)
                .with_k_a(shooter.constants.FLYWHEEL_VOLTAGE_K_A)
            )
            .with_slot1(
                # Use slot 1 for torque current FOC gains.
                phoenix6.configs.Slot1Configs()
                .with_k_p(shooter.constants.FLYWHEEL_CURRENT_K_P)
                .with_k_i(shooter.constants.FLYWHEEL_CURRENT_K_I)
                .with_k_d(shooter.constants.FLYWHEEL_CURRENT_K_D)
                .with_k_s(shooter.constants.FLYWHEEL_CURRENT_K_S)
                .with_k_v(shooter.constants.FLYWHEEL_CURRENT_K_V)
                .with_k_a(shooter.constants.FLYWHEEL_CURRENT_K_A)
            )
            .with_motion_magic(
                phoenix6.configs.MotionMagicConfigs()
                .with_motion_magic_acceleration(
                    shooter.constants.FLYWHEEL_MAX_ACCELERATION_RPS2
                )
                .with_motion_magic_jerk(
                    shooter.constants.FLYWHEEL_MAX_JERK_RPS3
                )
            )
        )

        # Configure the encoder.
        # https://api.ctr-electronics.com/phoenix6/stable/python/autoapi/phoenix6/configs/cancoder_configs/index.html
        self.flywheel_encoder.configurator.apply(
            phoenix6.configs.CANcoderConfiguration().with_magnet_sensor(
                phoenix6.configs.MagnetSensorConfigs().with_sensor_direction(
                    phoenix6.signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
                )
            )
        )

        self._target_rps: float = 0.0
        self._max_acceleration_rps2: float = (
            shooter.constants.FLYWHEEL_MAX_ACCELERATION_RPS2
        )
        self._control_type: FlywheelControlType = FlywheelControlType.VOLTAGE

        # Create requests for voltage and current control, using the appropriate
        # slots.
        self._voltage_request = phoenix6.controls.VelocityVoltage(
            self._target_rps
        ).with_slot(0)
        self._current_request = (
            phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC(
                self._target_rps
            ).with_slot(1)
        )

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        if self._control_type == FlywheelControlType.VOLTAGE:
            self.flywheel_motor.set_control(
                self._voltage_request.with_velocity(self._target_rps)
            )
        elif self._control_type == FlywheelControlType.TORQUE_CURRENT_FOC:
            self.flywheel_motor.set_control(
                self._current_request.with_velocity(
                    self._target_rps
                ).with_acceleration(self._max_acceleration_rps2)
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

    def setMaxAccelerationRps2(self, max_acceleration_rps2: float) -> None:
        self._max_acceleration_rps2 = max_acceleration_rps2

    def setControlType(self, control_type: FlywheelControlType) -> None:
        self._control_type = control_type

    @magicbot.feedback
    def get_target_rps(self) -> float:
        return self._target_rps

    @magicbot.feedback
    def get_control_type(self) -> int:
        return self._control_type.value


class FlywheelTuner:
    """Component for tuning the flywheel gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable flywheel target velocity.
    """

    flywheel_motor: phoenix6.hardware.TalonFX
    flywheel_encoder: phoenix6.hardware.CANcoder
    flywheel: Flywheel

    # Voltage control gains.
    voltage_k_p = magicbot.tunable(shooter.constants.FLYWHEEL_VOLTAGE_K_P)
    voltage_k_i = magicbot.tunable(shooter.constants.FLYWHEEL_VOLTAGE_K_I)
    voltage_k_d = magicbot.tunable(shooter.constants.FLYWHEEL_VOLTAGE_K_D)
    voltage_k_s = magicbot.tunable(shooter.constants.FLYWHEEL_VOLTAGE_K_S)
    voltage_k_v = magicbot.tunable(shooter.constants.FLYWHEEL_VOLTAGE_K_V)
    voltage_k_a = magicbot.tunable(shooter.constants.FLYWHEEL_VOLTAGE_K_A)

    # Torque current FOC gains.
    current_k_p = magicbot.tunable(shooter.constants.FLYWHEEL_CURRENT_K_P)
    current_k_i = magicbot.tunable(shooter.constants.FLYWHEEL_CURRENT_K_I)
    current_k_d = magicbot.tunable(shooter.constants.FLYWHEEL_CURRENT_K_D)
    current_k_s = magicbot.tunable(shooter.constants.FLYWHEEL_CURRENT_K_S)
    current_k_v = magicbot.tunable(shooter.constants.FLYWHEEL_CURRENT_K_V)
    current_k_a = magicbot.tunable(shooter.constants.FLYWHEEL_CURRENT_K_A)

    # The target rotational velocity of the flywheel.
    target_rps = magicbot.tunable(shooter.constants.FLYWHEEL_ENABLE_RPS)
    # The max acceleration allowed for the flywheel.
    max_acceleration_rps2 = magicbot.tunable(
        shooter.constants.FLYWHEEL_MAX_ACCELERATION_RPS2
    )
    # The control request type to use.
    control_type = magicbot.tunable(FlywheelControlType.VOLTAGE.value)

    def setup(self) -> None:
        """Set up initial state for the flywheel tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._voltage_gains = (
            phoenix6.configs.Slot0Configs()
            .with_k_p(shooter.constants.FLYWHEEL_VOLTAGE_K_P)
            .with_k_i(shooter.constants.FLYWHEEL_VOLTAGE_K_I)
            .with_k_d(shooter.constants.FLYWHEEL_VOLTAGE_K_D)
            .with_k_s(shooter.constants.FLYWHEEL_VOLTAGE_K_S)
            .with_k_v(shooter.constants.FLYWHEEL_VOLTAGE_K_V)
            .with_k_a(shooter.constants.FLYWHEEL_VOLTAGE_K_A)
        )
        self._current_gains = (
            phoenix6.configs.Slot1Configs()
            .with_k_p(shooter.constants.FLYWHEEL_CURRENT_K_P)
            .with_k_i(shooter.constants.FLYWHEEL_CURRENT_K_I)
            .with_k_d(shooter.constants.FLYWHEEL_CURRENT_K_D)
            .with_k_s(shooter.constants.FLYWHEEL_CURRENT_K_S)
            .with_k_v(shooter.constants.FLYWHEEL_CURRENT_K_V)
            .with_k_a(shooter.constants.FLYWHEEL_CURRENT_K_A)
        )

    def execute(self) -> None:
        """Update the flywheel speed and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.flywheel.setTargetRps(self.target_rps)
        self.flywheel.setMaxAccelerationRps2(self.max_acceleration_rps2)
        try:
            self.flywheel.setControlType(FlywheelControlType(self.control_type))
        except ValueError as e:
            self.logger.error(e)

        # We only want to reapply the gains if they changed. TalonFX doesn't
        # like being reconfigured constantly.
        if self.voltageGainsChanged():
            self.applyVoltageGains()
        if self.currentGainsChanged():
            self.applyCurrentGains()

    def voltageGainsChanged(self) -> bool:
        """Detect if any of the voltage control gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.voltage_k_p != self._voltage_gains.k_p
            or self.voltage_k_i != self._voltage_gains.k_i
            or self.voltage_k_d != self._voltage_gains.k_d
            or self.voltage_k_s != self._voltage_gains.k_s
            or self.voltage_k_v != self._voltage_gains.k_v
            or self.voltage_k_a != self._voltage_gains.k_a
        )

    def currentGainsChanged(self) -> bool:
        """Detect if any of the current control gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.current_k_p != self._current_gains.k_p
            or self.current_k_i != self._current_gains.k_i
            or self.current_k_d != self._current_gains.k_d
            or self.current_k_s != self._current_gains.k_s
            or self.current_k_v != self._current_gains.k_v
            or self.current_k_a != self._current_gains.k_a
        )

    def applyVoltageGains(self) -> None:
        """Apply the latest voltage gains to the motor."""
        result = self.flywheel_motor.configurator.apply(
            self._voltage_gains.with_k_s(self.voltage_k_s)
            .with_k_v(self.voltage_k_v)
            .with_k_a(self.voltage_k_a)
            .with_k_p(self.voltage_k_p)
            .with_k_i(self.voltage_k_i)
            .with_k_d(self.voltage_k_d)
        )
        if not result.is_ok():
            self.logger.error(
                f"Failed to apply new voltage gains to flywheel motor: {result.name}: {result.description}"
            )

    def applyCurrentGains(self) -> None:
        """Apply the latest current gains to the motor."""
        result = self.flywheel_motor.configurator.apply(
            self._current_gains.with_k_s(self.current_k_s)
            .with_k_v(self.current_k_v)
            .with_k_a(self.current_k_a)
            .with_k_p(self.current_k_p)
            .with_k_i(self.current_k_i)
            .with_k_d(self.current_k_d)
        )
        if not result.is_ok():
            self.logger.error(
                f"Failed to apply new current gains to flywheel motor: {result.name}: {result.description}"
            )

    @magicbot.feedback
    def get_motor_duty_cycle(self) -> float:
        return self.flywheel_motor.get_duty_cycle().value

    @magicbot.feedback
    def get_motor_voltage(self) -> phoenix6.units.volt:
        return self.flywheel_motor.get_motor_voltage().value

    @magicbot.feedback
    def get_motor_supply_voltage(self) -> phoenix6.units.volt:
        return self.flywheel_motor.get_supply_voltage().value

    @magicbot.feedback
    def get_motor_supply_current(self) -> phoenix6.units.ampere:
        return self.flywheel_motor.get_supply_current().value

    @magicbot.feedback
    def get_motor_stator_current(self) -> phoenix6.units.ampere:
        return self.flywheel_motor.get_stator_current().value

    @magicbot.feedback
    def get_measured_rps(self) -> float:
        return self.flywheel_encoder.get_velocity().value
