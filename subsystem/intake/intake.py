import magicbot
from phoenix6 import configs, controls, hardware

import constants

class Intake:
    """Intake component

    This class drives the intake motors that pick up fuel into the hopper.
    """

    robot_constants: constants.RobotConstants
    intake_roller_motor: hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the intake.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._active = False
        self._active_roller_speed_rps = (
            self.robot_constants.intake.active_roller_speed_rps
        )

        self.intake_roller_motor.configurator.apply(
            configs.TalonFXConfiguration()
            .with_motor_output(
                configs.MotorOutputConfigs().with_inverted(
                    self.robot_constants.intake.roller_motor_inverted
                )
            )
            .with_slot0(
                configs.Slot0Configs()
                .with_k_s(self.robot_constants.intake.k_s)
                .with_k_v(self.robot_constants.intake.k_v)
                .with_k_a(self.robot_constants.intake.k_a)
                .with_k_p(self.robot_constants.intake.k_p)
                .with_k_i(self.robot_constants.intake.k_i)
                .with_k_d(self.robot_constants.intake.k_d)
            )
        )

        self._request = controls.VelocityVoltage(0.0).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the requested speed.

        This method is called at the end of the control loop.
        """
        if self._active:
            self._request.with_velocity(self._active_roller_speed_rps)
        else:
            self._request.with_velocity(0.0)

        self.intake_roller_motor.set_control(self._request)

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._active = False

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._active = False

    def setSpeed(self, speed_rps: float = None) -> None:
        """Set the intake roller motor's speed."""
        self._active_roller_speed_rps = speed_rps

    def setActive(self, active: bool) -> None:
        """Set whether or not the intake roller is active."""
        self._active = active

    def toggleActive(self) -> None:
        """Toggle the intake roller between active and inactive."""
        self._active = not self._active

    @magicbot.feedback
    def get_measured_speed(self) -> float:
        value = self.intake_roller_motor.get_velocity().value
        return value if value else 0.0


class IntakeTuner:
    """Component for tuning the intake gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope.
    """

    robot_constants: constants.RobotConstants
    intake_roller_motor: hardware.TalonFX
    intake: Intake

    # Gains for velocity control of the intake.
    k_s = magicbot.tunable(0.0)
    k_v = magicbot.tunable(0.0)
    k_a = magicbot.tunable(0.0)
    k_p = magicbot.tunable(0.0)
    k_i = magicbot.tunable(0.0)
    k_d = magicbot.tunable(0.0)

    target_speed_rps = magicbot.tunable(0.0)
    active = magicbot.tunable(False)

    def setup(self) -> None:
        """Set up initial state for the intake tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        intake_constants = self.robot_constants.intake

        self.k_s = intake_constants.k_s
        self.k_v = intake_constants.k_v
        self.k_a = intake_constants.k_a
        self.k_p = intake_constants.k_p
        self.k_i = intake_constants.k_i
        self.k_d = intake_constants.k_d

        self.last_k_s = self.k_s
        self.last_k_v = self.k_v
        self.last_k_a = self.k_a
        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d

    def execute(self) -> None:
        """Update the intake speed and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.intake.setActive(self.active)
        self.intake.setSpeed(self.target_speed_rps)

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
        result = self.intake_roller_motor.configurator.apply(
            configs.config_groups.Slot0Configs()
            .with_k_s(self.k_s)
            .with_k_v(self.k_v)
            .with_k_a(self.k_a)
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to intake motor")
