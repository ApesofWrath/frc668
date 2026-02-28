import magicbot
import phoenix6

import constants
from subsystem import shooter


class Flywheel:
    """Flywheel component

    This class controls the rotational velocity of the flywheel.
    """

    robot_constants: constants.RobotConstants
    flywheel_motor: phoenix6.hardware.TalonFX
    flywheel_encoder: phoenix6.hardware.CANcoder

    def setup(self) -> None:
        """Set up initial state for the flywheel.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        flywheel_constants: shooter.FlywheelConstants = (
            self.robot_constants.shooter.flywheel
        )
        self.flywheel_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration()
            .with_feedback(
                phoenix6.configs.FeedbackConfigs()
                .with_feedback_sensor_source(
                    phoenix6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER
                )
                .with_feedback_remote_sensor_id(
                    flywheel_constants.encoder_can_id
                )
            )
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(flywheel_constants.motor_inverted)
                .with_neutral_mode(phoenix6.signals.NeutralModeValue.COAST)
            )
            .with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_p(flywheel_constants.k_p)
                .with_k_i(flywheel_constants.k_i)
                .with_k_d(flywheel_constants.k_d)
                .with_k_s(flywheel_constants.k_s)
                .with_k_v(flywheel_constants.k_v)
                .with_k_a(flywheel_constants.k_a)
            )
        )
        self.flywheel_encoder.configurator.apply(
            phoenix6.configs.CANcoderConfiguration().with_magnet_sensor(
                phoenix6.configs.MagnetSensorConfigs().with_sensor_direction(
                    flywheel_constants.encoder_direction
                )
            )
        )

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
        self._target_rps = (
            self.robot_constants.shooter.flywheel.default_speed_rps
        )

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

    robot_constants: constants.RobotConstants
    flywheel_motor: phoenix6.hardware.TalonFX
    flywheel_encoder: phoenix6.hardware.CANcoder
    flywheel: Flywheel

    # Gains for velocity control of the flywheel.
    k_s = magicbot.tunable(0.0)
    k_v = magicbot.tunable(0.0)
    k_a = magicbot.tunable(0.0)
    k_p = magicbot.tunable(0.0)
    k_i = magicbot.tunable(0.0)
    k_d = magicbot.tunable(0.0)

    # The target rotational velocity of the flywheel.
    target_rps = magicbot.tunable(0.0)

    def setup(self) -> None:
        """Set up initial state for the flywheel tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        flywheel_constants: shooter.FlywheelConstants = (
            self.robot_constants.shooter.flywheel
        )

        self.k_s = flywheel_constants.k_s
        self.k_v = flywheel_constants.k_v
        self.k_a = flywheel_constants.k_a
        self.k_p = flywheel_constants.k_p
        self.k_i = flywheel_constants.k_i
        self.k_d = flywheel_constants.k_d

        self.last_k_s = self.k_s
        self.last_k_v = self.k_v
        self.last_k_a = self.k_a
        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d

        self.target_rps = flywheel_constants.default_speed_rps

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
        result = self.flywheel_motor.configurator.apply(
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.k_s)
            .with_k_v(self.k_v)
            .with_k_a(self.k_a)
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to flywheel motor")

    @magicbot.feedback
    def get_motor_voltage(self) -> phoenix6.units.volt:
        return self.flywheel_motor.get_motor_voltage().value

    @magicbot.feedback
    def get_motor_stator_current(self) -> phoenix6.units.ampere:
        return self.flywheel_motor.get_stator_current().value

    @magicbot.feedback
    def get_measured_rps(self) -> float:
        return self.flywheel_encoder.get_velocity().value
