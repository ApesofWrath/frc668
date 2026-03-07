import phoenix6
import magicbot
from subsystem.intake.constants import INTAKE_CONSTANTS, DEPLOY_INTAKE_CONSTANTS
import constants
from phoenix6.controls import MotionMagicDutyCycle
from phoenix6.configs import config_groups
phoenix6.controls.position_duty_cycle

class Intake:
    """Intake component

    This class drives the intake motors that pick up fuel into the hopper.
    """

    robot_constants: constants.RobotConstants
    intake_roller_motor: phoenix6.hardware.TalonFX
    intake_deploy_motor: phoenix6.hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the intake.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._motor_speed = 0.0
        self._deploy_motor_speed = 0.0

        self.intake_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration().with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    self.robot_constants.intake.motor_inverted
                )
            )
        )
        deploy_intake_constants = self.robot_constants.intake

        self.deploy_intake_encoder_configs = (
            phoenix6.configs.CANcoderConfiguration().with_magnet_sensor(
                phoenix6.configs.MagnetSensorConfigs()
                .with_sensor_direction(deploy_intake_constants.encoder_direction)
                .with_absolute_sensor_discontinuity_point(
                    deploy_intake_constants.absolute_sensor_discontinuity_point
                )
            )
        )
        self.intake_deploy_constant_configs = (
            phoenix6.configs.TalonFXConfiguration()
            .with_feedback(
                phoenix6.configs.FeedbackConfigs()
                .with_feedback_sensor_source(phoenix6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
                .with_feedback_remote_sensor_id(deploy_intake_constants.deploy_encoder_can_id)
                .with_sensor_to_mechanism_ratio(deploy_intake_constants.sensor_to_mechanism_ratio)
                .with_rotor_to_sensor_ratio(deploy_intake_constants.rotor_to_sensor_ratio)
            )

            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    deploy_intake_constants.motor_inverted
                )
            )
            .with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_s(deploy_intake_constants.position_k_s)
                .with_k_v(deploy_intake_constants.position_k_v)
                .with_k_a(deploy_intake_constants.position_k_a)
                .with_k_p(deploy_intake_constants.position_k_p)
                .with_k_i(deploy_intake_constants.position_k_i)
                .with_k_d(deploy_intake_constants.position_k_d)
            )
            .with_motion_magic(
                phoenix6.configs.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(
                    deploy_intake_constants.motion_magic_cruise_velocity
                )
                .with_motion_magic_acceleration(
                    deploy_intake_constants.motion_magic_acceleration
                )
                .with_motion_magic_jerk(deploy_intake_constants.motion_magic_jerk)
            )
        )
    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.intake_motor.set(self._motor_speed)
        self.intake_deploy_motor.set(self._deploy_motor_speed)

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

    def setMotorSpeed(self, speed: float) -> None:
        self._motor_speed = speed




class DeployIntakeTuner:
    robot_constants: constants.RobotConstants
    intake_deploy_motor: phoenix6.hardware.TalonFX
    deploy_intake_encoder: phoenix6.hardware.CANcoder
    intake: Intake

    # Gains for position control of the deploy intake.
    position_k_p = magicbot.tunable(0.0)
    position_k_i = magicbot.tunable(0.0)
    position_k_d = magicbot.tunable(0.0)

    # Limits for motion magic.
    mm_cruise_velocity = magicbot.tunable(0.0)
    mm_acceleration = magicbot.tunable(0.0)
    mm_jerk = magicbot.tunable(0.0)

    # Feedforward for motion magic.
    mm_feed_forward = magicbot.tunable(0.0)

    # The target position of the deploy intake.
    target_position = magicbot.tunable(0.0)
    
    def setup(self) -> None:
        """Set up initial state for the intake tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        deploy_intake_constants = DEPLOY_INTAKE_CONSTANTS["0323CA4B"]

        self.position_k_p = deploy_intake_constants.position_k_p
        self.position_k_i = deploy_intake_constants.position_k_i
        self.position_k_d = deploy_intake_constants.position_k_d

        self.mm_cruise_velocity = deploy_intake_constants.motion_magic_cruise_velocity
        self.mm_acceleration = deploy_intake_constants.motion_magic_acceleration
        self.mm_jerk = deploy_intake_constants.motion_magic_jerk

        self.mm_feed_forward = deploy_intake_constants.motion_magic_feed_forward

        self.last_position_k_p = self.position_k_p
        self.last_position_k_i = self.position_k_i
        self.last_position_k_d = self.position_k_d


        self.last_mm_cruise_velocity = self.mm_cruise_velocity
        self.last_mm_acceleration = self.mm_acceleration
        self.last_mm_jerk = self.mm_jerk

        self.logger.info("DeployIntakeTuner initialized")

    def execute(self) -> None:
        """Update the deploy intake position and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.intake.setPosition(self.target_position)
        self.intake.setMotionMagicFeedForward(self.mm_feed_forward)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if not self.gainsChanged():
            return

        self.applyGains()

        self.last_position_k_p = self.position_k_p
        self.last_position_k_i = self.position_k_i
        self.last_position_k_d = self.position_k_d


        self.last_mm_cruise_velocity = self.mm_cruise_velocity
        self.last_mm_acceleration = self.mm_acceleration
        self.last_mm_jerk = self.mm_jerk

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.position_k_p != self.last_position_k_p
            or self.position_k_i != self.last_position_k_i
            or self.position_k_d != self.last_position_k_d
            or self.mm_cruise_velocity != self.last_mm_cruise_velocity
            or self.mm_acceleration != self.last_mm_acceleration
            or self.mm_jerk != self.last_mm_jerk
        )

    def applyGains(self) -> None:
        """Apply the current gains to the motor."""
        deploy_intake_constants = DEPLOY_INTAKE_CONSTANTS["0323CA4B"]
        self.logger.info("Applying deploy intake gains...")
        slot0_configs = (
            phoenix6.configs.Slot0Configs()
            .with_k_p(self.position_k_p)
            .with_k_i(self.position_k_i)
            .with_k_d(self.position_k_d)
        )
        motion_magic_configs = (
            phoenix6.configs.MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(self.mm_cruise_velocity)
            .with_motion_magic_acceleration(self.mm_acceleration)
            .with_motion_magic_jerk(self.mm_jerk)
        )
        result = self.intake_deploy_motor.configurator.apply(
            self.intake.intake_deploy_motor_configs.with_slot0(slot0_configs)
            .with_motion_magic(motion_magic_configs)
        )
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to deploy intake motor")

        result = self.deploy_intake_encoder.configurator.apply(


        )
