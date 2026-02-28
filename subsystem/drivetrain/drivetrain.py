import logging
import typing

import magicbot
import wpilib
import wpimath
from phoenix6 import hardware, swerve, units

from common import joystick
from subsystem import drivetrain


class Drivetrain(swerve.SwerveDrivetrain):
    def __init__(self, constants: drivetrain.DrivetrainConstants):
        constants_factory: swerve.SwerveModuleConstantsFactory[
            configs.TalonFXConfiguration,
            configs.TalonFXConfiguration,
            configs.CANcoderConfiguration,
        ] = (
            swerve.SwerveModuleConstantsFactory()
            .with_drive_motor_gear_ratio(
                constants.common.drive_motor_gear_ratio
            )
            .with_steer_motor_gear_ratio(
                constants.common.steer_motor_gear_ratio
            )
            .with_coupling_gear_ratio(constants.common.coupling_gear_ratio)
            .with_wheel_radius(constants.common.wheel_radius)
            .with_steer_motor_gains(constants.common.steer_motor_gains)
            .with_drive_motor_gains(constants.common.drive_motor_gains)
            .with_steer_motor_closed_loop_output(
                constants.common.steer_motor_closed_loop_output
            )
            .with_drive_motor_closed_loop_output(
                constants.common.drive_motor_closed_loop_output
            )
            .with_slip_current(constants.common.slip_current)
            .with_speed_at12_volts(constants.common.speed_at12_volts)
            .with_drive_motor_type(constants.common.drive_motor_type)
            .with_steer_motor_type(constants.common.steer_motor_type)
            .with_feedback_source(constants.common.feedback_source)
            .with_drive_motor_initial_configs(
                constants.common.drive_motor_initial_configs
            )
            .with_steer_motor_initial_configs(
                constants.common.steer_motor_initial_configs
            )
            .with_encoder_initial_configs(
                constants.common.encoder_initial_configs
            )
        )

        super().__init__(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            swerve.SwerveDrivetrainConstants()
            .with_can_bus_name(constants.drivetrain.can_bus_name)
            .with_pigeon2_id(constants.drivetrain.pigeon2_id)
            .with_pigeon2_configs(constants.drivetrain.pigeon2_configs),
            [
                constants_factory.create_module_constants(
                    constants.front_left.steer_motor_id,
                    constants.front_left.drive_motor_id,
                    constants.front_left.encoder_id,
                    constants.front_left.encoder_offset,
                    constants.front_left.location_x,
                    constants.front_left.location_y,
                    constants.front_left.drive_motor_inverted,
                    constants.front_left.steer_motor_inverted,
                    constants.front_left.encoder_inverted,
                ),
                constants_factory.create_module_constants(
                    constants.front_right.steer_motor_id,
                    constants.front_right.drive_motor_id,
                    constants.front_right.encoder_id,
                    constants.front_right.encoder_offset,
                    constants.front_right.location_x,
                    constants.front_right.location_y,
                    constants.front_right.drive_motor_inverted,
                    constants.front_right.steer_motor_inverted,
                    constants.front_right.encoder_inverted,
                ),
                constants_factory.create_module_constants(
                    constants.back_left.steer_motor_id,
                    constants.back_left.drive_motor_id,
                    constants.back_left.encoder_id,
                    constants.back_left.encoder_offset,
                    constants.back_left.location_x,
                    constants.back_left.location_y,
                    constants.back_left.drive_motor_inverted,
                    constants.back_left.steer_motor_inverted,
                    constants.back_left.encoder_inverted,
                ),
                constants_factory.create_module_constants(
                    constants.back_right.steer_motor_id,
                    constants.back_right.drive_motor_id,
                    constants.back_right.encoder_id,
                    constants.back_right.encoder_offset,
                    constants.back_right.location_x,
                    constants.back_right.location_y,
                    constants.back_right.drive_motor_inverted,
                    constants.back_right.steer_motor_inverted,
                    constants.back_right.encoder_inverted,
                ),
            ],
        )

        self.logger = logging.getLogger(__name__)
        self._alliance: typing.Optional[wpilib.DriverStation.Alliance] = None

    def setup(self) -> None:
        """Set up initial state for the drivetrain.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # Try to apply operator perspective forward based on alliance.
        self.maybeSetOperatorPerspectiveForward()

        self._drive_request = (
            swerve.requests.FieldCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

    def execute(self) -> None:
        """Command the drivetrain to the current speeds.

        This method is called at the end of the control loop.
        """
        if not self._alliance:
            self.logger.error(
                "Failed to apply operator prespective based on alliance."
            )
        self.set_control(self._drive_request)

    def maybeSetOperatorPerspectiveForward(self) -> None:
        # If we have our alliance, we already set operator perspective.
        if self._alliance:
            return
        # If not, try to get the alliance.
        self._alliance = wpilib.DriverStation.getAlliance()
        # If we got it, set the operator perspective.
        if self._alliance:
            self.logger.info(f"We are alliance: {self._alliance}")
            self.set_operator_perspective_forward(
                drivetrain.constants.RED_ALLIANCE_PERSPECTIVE_ROTATION
                if self._alliance == wpilib.DriverStation.Alliance.kRed
                else drivetrain.constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
            )

    def setSpeeds(
        self,
        command: joystick.DriveCommand,
    ) -> None:
        self._drive_request.with_velocity_x(command.vx).with_velocity_y(
            command.vy
        ).with_rotational_rate(command.omega)

    def isManual(self):
        return True

    @magicbot.feedback
    def get_robot_pose(self) -> wpimath.geometry.Pose2d:
        return self.get_state().pose
