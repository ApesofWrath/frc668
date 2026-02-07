import magicbot
import phoenix6

from subsystem import shooter


class Flywheel:
    """Flywheel component

    This class controls the rotational velocity of the flywheel.
    """

    flywheel_encoder: phoenix6.hardware.CANcoder
    flywheel_motor: phoenix6.hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the flywheel.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # The target rotational velocity of the flywheel.
        self._target_rps = 0.0

        # Configure the feedback sensor to use and the feedforward + feedback
        # gains.
        self._flywheel_configs = phoenix6.configs.TalonFXConfiguration()
        self._flywheel_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        self._flywheel_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.FLYWHEEL_ENCODER_CAN_ID
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
            self._request.with_velocity(-self._target_rps)
        )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self.logger.info("ENABLED")
        self._target_rps = shooter.constants.FLYWHEEL_ENABLE_RPS

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._target_rps = 0.0

    def setTargetRps(self, rotations_per_second: float) -> None:
        """Set the target rotational velocity of the flywheel.

        Args:
            rotations_per_second: A decimal target rotational velocity for the
                flywheel, in rotations per second.
        """
        self._target_rps = rotations_per_second

    @magicbot.feedback
    def get_target_rps(self) -> float:
        return self._target_rps

    @magicbot.feedback
    def get_flywheel_rps(self) -> float:
        return self.flywheel_encoder.get_velocity().value

    def _setSlot0Configs(
        self,
        new_configs: phoenix6.configs.config_groups.Slot0Configs,
    ) -> None:
        """Set a new set of feedback gains for the motor.

        Note that this method must *not* be called periodically, and must
        generally only be used for tuning the gains of this mechanism.

        Args:
            new_feedback: The new set of feedforward/feedback gains for the
                flywheel motor to use.
        """
        self._flywheel_configs = self._flywheel_configs.with_slot0(new_configs)
        self.flywheel_motor.configurator.apply(self._flywheel_configs)
