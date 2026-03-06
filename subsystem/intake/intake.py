import phoenix6

from subsystem.intake.constants import INTAKE_CONSTANTS, DEPLOY_INTAKE_CONSTANTS


class Intake:
    """Intake component

    This class drives the intake motors that pick up fuel into the hopper.
    """

    robot_constants: constants.RobotConstants
    intake_motor: phoenix6.hardware.TalonFX
    deploy_intake_motor: phoenix6.hardware.TalonFX

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


        self.deploy_intake_motor_configs = (
            phoenix6.configs.TalonFXConfiguration()
            .with_feedback(
                phoenix6.configs.FeedbackConfigs()
                .with_feedback_sensor_source(phoenix6.signals.FeedbackSensorSourceValue.REMOTE_CANCODER)
                .with_feedback_remote_sensor_id(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].deploy_encoder_can_id)
                .with_sensor_to_mechanism_ratio(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].sensor_to_mechanism_ratio)
                .with_rotor_to_sensor_ratio(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].rotor_to_sensor_ratio)
            )
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    DEPLOY_INTAKE_CONSTANTS["0323CA4B"].motor_inverted
                )
            )
            .with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_s(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].position_k_s)
                .with_k_v(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].position_k_v)
                .with_k_a(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].position_k_a)
                .with_k_p(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].position_k_p)
                .with_k_i(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].position_k_i)
                .with_k_d(DEPLOY_INTAKE_CONSTANTS["0323CA4B"].position_k_d)
            )
            .with_slot1(
                phoenix6.configs.Slot1Configs()
                .with_k_s(turret_constants.velocity_k_s)
                .with_k_v(turret_constants.velocity_k_v)
                .with_k_a(turret_constants.velocity_k_a)
                .with_k_p(turret_constants.velocity_k_p)
                .with_k_i(turret_constants.velocity_k_i)
                .with_k_d(turret_constants.velocity_k_d)
            )
            .with_motion_magic(
                phoenix6.configs.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(
                    turret_constants.motion_magic_cruise_velocity
                )
                .with_motion_magic_acceleration(
                    turret_constants.motion_magic_acceleration
                )
                .with_motion_magic_jerk(turret_constants.motion_magic_jerk)
            )
        )
    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.intake_motor.set(self._motor_speed)
        self.deploy_intake_motor.set(self._deploy_motor_speed)

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

    def setDeployMotorSpeed(self, speed: float) -> None:
        self._deploy_motor_speed = speed
