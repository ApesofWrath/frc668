import magicbot
import phoenix6

import constants
from subsystem import shooter


class Hopper:
    """Hopper component

    This class drives the hopper motors that feed fuel from the hopper to the
    indexer.
    """

    robot_constants: constants.RobotConstants
    hopper_left_motor: phoenix6.hardware.TalonFX
    hopper_right_motor: phoenix6.hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the hopper.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        hopper_constants: shooter.HopperConstants = (
            self.robot_constants.shooter.hopper
        )
        self.hopper_left_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration()
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    hopper_constants.left_motor_inverted
                )
            )
            .with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_s(hopper_constants.left_k_s)
                .with_k_v(hopper_constants.left_k_v)
                .with_k_a(hopper_constants.left_k_a)
                .with_k_p(hopper_constants.left_k_p)
                .with_k_i(hopper_constants.left_k_i)
                .with_k_d(hopper_constants.left_k_d)
            )
        )
        self.hopper_right_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration()
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    hopper_constants.right_motor_inverted
                )
            )
            .with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_s(hopper_constants.right_k_s)
                .with_k_v(hopper_constants.right_k_v)
                .with_k_a(hopper_constants.right_k_a)
                .with_k_p(hopper_constants.right_k_p)
                .with_k_i(hopper_constants.right_k_i)
                .with_k_d(hopper_constants.right_k_d)
            )
        )

        # The target speed (in rotations per second) to request the hopper
        # motors to run at.
        self._target_rps: float = 0.0
        # The hopper motors are run when this is True.
        self._enabled: bool = False

        self._request = phoenix6.controls.VelocityVoltage(
            self._target_rps
        ).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        if self._enabled:
            self.hopper_left_motor.set_control(
                self._request.with_velocity(self._target_rps)
            )
            self.hopper_right_motor.set_control(
                self._request.with_velocity(self._target_rps)
            )
        else:
            self.hopper_left_motor.set_control(self._request.with_velocity(0.0))
            self.hopper_right_motor.set_control(
                self._request.with_velocity(0.0)
            )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._target_rps = self.robot_constants.shooter.hopper.default_speed_rps
        self._enabled = False

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._target_rps = 0.0
        self._enabled = False

    def setTargetRps(self, target_rps: float) -> None:
        """Set the target speed for both hopper motors.

        Args:
            target_rps: Speed in rotations per second.
        """
        self._target_rps = target_rps

    def setEnabled(self, value: bool) -> None:
        """Enable or disable the hopper motors.

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


class HopperTuner:
    """Component for tuning hopper gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable hopper target speed.
    """

    robot_constants: constants.RobotConstants
    hopper_left_motor: phoenix6.hardware.TalonFX
    hopper_right_motor: phoenix6.hardware.TalonFX
    hopper: Hopper

    # Gains for velocity control of the left hopper motor.
    left_k_s = magicbot.tunable(0.0)
    left_k_v = magicbot.tunable(0.0)
    left_k_a = magicbot.tunable(0.0)
    left_k_p = magicbot.tunable(0.0)
    left_k_i = magicbot.tunable(0.0)
    left_k_d = magicbot.tunable(0.0)

    # Gains for velocity control of the right hopper motor.
    right_k_s = magicbot.tunable(0.0)
    right_k_v = magicbot.tunable(0.0)
    right_k_a = magicbot.tunable(0.0)
    right_k_p = magicbot.tunable(0.0)
    right_k_i = magicbot.tunable(0.0)
    right_k_d = magicbot.tunable(0.0)

    # The target rotational speed of the hopper.
    target_rps = magicbot.tunable(0.0)
    # Whether or not the hopper motors should run.
    enabled = magicbot.tunable(False)

    def setup(self) -> None:
        hopper_constants: shooter.HopperConstants = (
            self.robot_constants.shooter.hopper
        )

        self.left_k_s = hopper_constants.left_k_s
        self.left_k_v = hopper_constants.left_k_v
        self.left_k_a = hopper_constants.left_k_a
        self.left_k_p = hopper_constants.left_k_p
        self.left_k_i = hopper_constants.left_k_i
        self.left_k_d = hopper_constants.left_k_d

        self.right_k_s = hopper_constants.right_k_s
        self.right_k_v = hopper_constants.right_k_v
        self.right_k_a = hopper_constants.right_k_a
        self.right_k_p = hopper_constants.right_k_p
        self.right_k_i = hopper_constants.right_k_i
        self.right_k_d = hopper_constants.right_k_d

        self._current_left_gains = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.left_k_s)
            .with_k_v(self.left_k_v)
            .with_k_a(self.left_k_a)
            .with_k_p(self.left_k_p)
            .with_k_i(self.left_k_i)
            .with_k_d(self.left_k_d)
        )
        self._current_right_gains = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.right_k_s)
            .with_k_v(self.right_k_v)
            .with_k_a(self.right_k_a)
            .with_k_p(self.right_k_p)
            .with_k_i(self.right_k_i)
            .with_k_d(self.right_k_d)
        )

    def execute(self) -> None:
        self.hopper.setTargetRps(self.target_rps)
        self.hopper.setEnabled(self.enabled)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if self.leftGainsChanged():
            self.applyLeftGains()
        if self.rightGainsChanged():
            self.applyRightGains()

    def leftGainsChanged(self) -> bool:
        """Detect if any of the gains for the left motor changed.

        Returns:
            True if any of the gains changed, False if they didn't.
        """
        return (
            self.left_k_s != self._current_left_gains.k_s
            or self.left_k_v != self._current_left_gains.k_v
            or self.left_k_a != self._current_left_gains.k_a
            or self.left_k_p != self._current_left_gains.k_p
            or self.left_k_i != self._current_left_gains.k_i
            or self.left_k_d != self._current_left_gains.k_d
        )

    def rightGainsChanged(self) -> bool:
        """Detect if any of the gains for the right motor changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.right_k_s != self._current_right_gains.k_s
            or self.right_k_v != self._current_right_gains.k_v
            or self.right_k_a != self._current_right_gains.k_a
            or self.right_k_p != self._current_right_gains.k_p
            or self.right_k_i != self._current_right_gains.k_i
            or self.right_k_d != self._current_right_gains.k_d
        )

    def applyLeftGains(self) -> None:
        """Apply the current gains to the left motor."""
        result = self.hopper_left_motor.configurator.apply(
            self._current_left_gains.with_k_s(self.left_k_s)
            .with_k_v(self.left_k_v)
            .with_k_a(self.left_k_a)
            .with_k_p(self.left_k_p)
            .with_k_i(self.left_k_i)
            .with_k_d(self.left_k_d)
        )
        if not result.is_ok():
            self.logger.error(
                (
                    f"Failed to apply new gains to hopper left motor: "
                    f"{result.name}: {result.description}"
                )
            )

    def applyRightGains(self) -> None:
        """Apply the current gains to the right motor."""
        result = self.hopper_right_motor.configurator.apply(
            self._current_right_gains.with_k_s(self.right_k_s)
            .with_k_v(self.right_k_v)
            .with_k_a(self.right_k_a)
            .with_k_p(self.right_k_p)
            .with_k_i(self.right_k_i)
            .with_k_d(self.right_k_d)
        )
        if not result.is_ok():
            self.logger.error(
                (
                    f"Failed to apply new gains to hopper right motor: "
                    f"{result.name}: {result.description}"
                )
            )

    @magicbot.feedback
    def get_left_measured_rps(self) -> float:
        return self.hopper_left_motor.get_velocity().value

    @magicbot.feedback
    def get_right_measured_rps(self) -> float:
        return self.hopper_right_motor.get_velocity().value
