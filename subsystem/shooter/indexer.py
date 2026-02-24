import magicbot
import phoenix6

from subsystem import shooter


class Indexer:
    """Indexer component

    This class drives the indexer motors that feed fuel into the flywheel.
    """

    indexer_back_motor: phoenix6.hardware.TalonFX
    indexer_front_motor: phoenix6.hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the indexer.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # Configuration settings for back motor.
        back_motor_configs = phoenix6.configs.TalonFXConfiguration()
        back_motor_configs.slot0.k_s = shooter.constants.INDEXER_BACK_K_S
        back_motor_configs.slot0.k_v = shooter.constants.INDEXER_BACK_K_V
        back_motor_configs.slot0.k_a = shooter.constants.INDEXER_BACK_K_A
        back_motor_configs.slot0.k_p = shooter.constants.INDEXER_BACK_K_P
        back_motor_configs.slot0.k_i = shooter.constants.INDEXER_BACK_K_I
        back_motor_configs.slot0.k_d = shooter.constants.INDEXER_BACK_K_D
        back_motor_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.indexer_back_motor.configurator.apply(back_motor_configs)

        # Configuration settings for front motor.
        front_motor_configs = phoenix6.configs.TalonFXConfiguration()
        front_motor_configs.slot0.k_s = shooter.constants.INDEXER_FRONT_K_S
        front_motor_configs.slot0.k_v = shooter.constants.INDEXER_FRONT_K_V
        front_motor_configs.slot0.k_a = shooter.constants.INDEXER_FRONT_K_A
        front_motor_configs.slot0.k_p = shooter.constants.INDEXER_FRONT_K_P
        front_motor_configs.slot0.k_i = shooter.constants.INDEXER_FRONT_K_I
        front_motor_configs.slot0.k_d = shooter.constants.INDEXER_FRONT_K_D
        front_motor_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.indexer_front_motor.configurator.apply(front_motor_configs)

        # The target speed (in rotations per second) to request the indexer
        # motors to run at.
        self._target_rps: float = 0.0
        # The indexer motors are run when this is True.
        self._enabled: bool = False

        self._request = phoenix6.controls.VelocityVoltage(
            self._target_rps
        ).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the current speed if enabled.

        This method is called at the end of the control loop.
        """
        if self._enabled:
            self.indexer_back_motor.set_control(
                self._request.with_velocity(self._target_rps)
            )
            self.indexer_front_motor.set_control(
                self._request.with_velocity(self._target_rps)
            )
        else:
            self.indexer_back_motor.set_control(
                self._request.with_velocity(0.0)
            )
            self.indexer_front_motor.set_control(
                self._request.with_velocity(0.0)
            )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._target_rps = shooter.constants.INDEXER_MOTORS_RPS_DEFAULT
        self._enabled = False

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._target_rps = 0.0
        self._enabled = False

    def setTargetRps(self, target_rps: float) -> None:
        """Set the target speed for both indexer motors.

        Args:
            target_rps: Speed in rotations per second.
        """
        self._target_rps = target_rps

    def setEnabled(self, value: bool) -> None:
        """Enable or disable the indexer motors.

        Args:
            value: Set to True to enable the motors, False to disable them.
        """
        self._enabled = value

    @magicbot.feedback
    def get_target_rps(self) -> float:
        return self._target_rps

    @magicbot.feedback
    def get_enabled(self) -> bool:
        return self._enabled


class IndexerTuner:
    """Component for tuning indexer gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable indexer target speed.
    """

    indexer_back_motor: phoenix6.hardware.TalonFX
    indexer_front_motor: phoenix6.hardware.TalonFX
    indexer: Indexer

    # Gains for velocity control of the back indexer motor.
    back_k_s = magicbot.tunable(shooter.constants.INDEXER_BACK_K_S)
    back_k_v = magicbot.tunable(shooter.constants.INDEXER_BACK_K_V)
    back_k_a = magicbot.tunable(shooter.constants.INDEXER_BACK_K_A)
    back_k_p = magicbot.tunable(shooter.constants.INDEXER_BACK_K_P)
    back_k_i = magicbot.tunable(shooter.constants.INDEXER_BACK_K_I)
    back_k_d = magicbot.tunable(shooter.constants.INDEXER_BACK_K_D)

    # Gains for velocity control of the front indexer motor.
    front_k_s = magicbot.tunable(shooter.constants.INDEXER_FRONT_K_S)
    front_k_v = magicbot.tunable(shooter.constants.INDEXER_FRONT_K_V)
    front_k_a = magicbot.tunable(shooter.constants.INDEXER_FRONT_K_A)
    front_k_p = magicbot.tunable(shooter.constants.INDEXER_FRONT_K_P)
    front_k_i = magicbot.tunable(shooter.constants.INDEXER_FRONT_K_I)
    front_k_d = magicbot.tunable(shooter.constants.INDEXER_FRONT_K_D)

    # The target rotational speed of the indexer.
    target_rps = magicbot.tunable(0.0)
    # Whether or not the indexer motors should run.
    enabled = magicbot.tunable(False)

    def setup(self) -> None:
        self._current_back_gains = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.back_k_s)
            .with_k_v(self.back_k_v)
            .with_k_a(self.back_k_a)
            .with_k_p(self.back_k_p)
            .with_k_i(self.back_k_i)
            .with_k_d(self.back_k_d)
        )
        self._current_front_gains = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.front_k_s)
            .with_k_v(self.front_k_v)
            .with_k_a(self.front_k_a)
            .with_k_p(self.front_k_p)
            .with_k_i(self.front_k_i)
            .with_k_d(self.front_k_d)
        )

    def execute(self) -> None:
        self.indexer.setTargetRps(self.target_rps)
        self.indexer.setEnabled(self.enabled)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if self.backGainsChanged():
            self.applyBackGains()
        if self.frontGainsChanged():
            self.applyFrontGains()

    def backGainsChanged(self) -> bool:
        """Detect if any of the gains for the back motor changed.

        Returns:
            True if any of the gains changed, False if the gains didn't change
                or if the current gains couldn't be read from the motor.
        """
        return (
            self.back_k_s != self._current_back_gains.k_s
            or self.back_k_v != self._current_back_gains.k_v
            or self.back_k_a != self._current_back_gains.k_a
            or self.back_k_p != self._current_back_gains.k_p
            or self.back_k_i != self._current_back_gains.k_i
            or self.back_k_d != self._current_back_gains.k_d
        )

    def frontGainsChanged(self) -> bool:
        """Detect if any of the gains for the front motor changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.front_k_s != self._current_front_gains.k_s
            or self.front_k_v != self._current_front_gains.k_v
            or self.front_k_a != self._current_front_gains.k_a
            or self.front_k_p != self._current_front_gains.k_p
            or self.front_k_i != self._current_front_gains.k_i
            or self.front_k_d != self._current_front_gains.k_d
        )

    def applyBackGains(self) -> None:
        """Apply the current gains to the back motor."""
        result = self.indexer_back_motor.configurator.apply(
            self._current_back_gains.with_k_s(self.back_k_s)
            .with_k_v(self.back_k_v)
            .with_k_a(self.back_k_a)
            .with_k_p(self.back_k_p)
            .with_k_i(self.back_k_i)
            .with_k_d(self.back_k_d)
        )
        if not result.is_ok():
            self.logger.error(
                f"Failed to apply new gains to indexer back motor: {result.name}: {result.description}"
            )

    def applyFrontGains(self) -> None:
        """Apply the current gains to the front motor."""
        result = self.indexer_front_motor.configurator.apply(
            self._current_front_gains.with_k_s(self.front_k_s)
            .with_k_v(self.front_k_v)
            .with_k_a(self.front_k_a)
            .with_k_p(self.front_k_p)
            .with_k_i(self.front_k_i)
            .with_k_d(self.front_k_d)
        )
        if not result.is_ok():
            self.logger.error(
                f"Failed to apply new gains to indexer front motor: {result.name}: {result.description}"
            )

    @magicbot.feedback
    def get_back_measured_rps(self) -> float:
        return self.indexer_back_motor.get_velocity().value

    @magicbot.feedback
    def get_front_measured_rps(self) -> float:
        return self.indexer_front_motor.get_velocity().value
