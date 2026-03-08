import math
import phoenix6
import magicbot

import constants
from phoenix6 import swerve
from subsystem import drivetrain


class Intake:
    """Intake component

    This class drives the intake motors that pick up fuel into the hopper.
    """

    robot_constants: constants.RobotConstants
    intake_motor: phoenix6.hardware.TalonFX
    drivetrain: drivetrain.Drivetrain

    def setup(self) -> None:
        """Set up initial state for the intake.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._motor_speed = 0.0

        self.intake_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration().with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    self.robot_constants.intake.motor_inverted
                )
            ).with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_s(self.robot_constants.intake.k_s)
                .with_k_v(self.robot_constants.intake.k_v)
                .with_k_a(self.robot_constants.intake.k_a)
                .with_k_p(self.robot_constants.intake.k_p)
                .with_k_i(self.robot_constants.intake.k_i)
                .with_k_d(self.robot_constants.intake.k_d)
            )
        )

        self._request = phoenix6.controls.VelocityVoltage(
            self._motor_speed
        ).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the requested speed.

        This method is called at the end of the control loop.
        """
        self.intake_motor.set_control(self._request.with_velocity(self._motor_speed))

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._motor_speed = 0.0

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._motor_speed = 0.0
        
    def set_max_speed(self) -> None:
        """Set the intake motor's linear speed to twice the robot's max speed (6 m/s).
        """
        max_robot_speed = self.robot_constants.drivetrain.max_linear_speed_meters_per_second
        self._motor_speed = 2 * max_robot_speed
    
    def setSpeed(self, override_speed_rps: float = None) -> None:
        """Set the intake motor's linear speed to twice the robot's current speed.
        """
        if override_speed_rps:
            self._motor_speed = override_speed_rps
        else:
            current_robot_speed = self.drivetrain.get_robot_speed()
            self._motor_speed = max(2 * current_robot_speed, self.robot_constants.intake.min_intake_speed)

    @magicbot.feedback
    def get_measured_speed(self) -> float:
        value = self.intake_motor.get_velocity().value
        return value if value else 0.0

class IntakeTuner:
    """Component for tuning the intake gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. 
    """

    robot_constants: constants.RobotConstants
    intake_motor: phoenix6.hardware.TalonFX
    intake: Intake

    # Gains for velocity control of the intake.
    k_s = magicbot.tunable(0.0)
    k_v = magicbot.tunable(0.0)
    k_a = magicbot.tunable(0.0)
    k_p = magicbot.tunable(0.0)
    k_i = magicbot.tunable(0.0)
    k_d = magicbot.tunable(0.0)
    target_speed_rps = magicbot.tunable(0.0)

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
        result = self.intake_motor.configurator.apply(
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.k_s)
            .with_k_v(self.k_v)
            .with_k_a(self.k_a)
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to intake motor")
