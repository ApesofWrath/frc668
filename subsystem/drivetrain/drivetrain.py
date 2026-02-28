import logging
import wpilib
from phoenix6 import configs, hardware, swerve

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

    def setup(self) -> None:
        """Apply the operator perspective based on alliance color."""
        self.logger.info(f"DriverStation attached: {wpilib.DriverStation.isDSAttached()}")
        alliance_color = wpilib.DriverStation.getAlliance()
        if alliance_color is None:
            self.logger.error("Failed to get alliance from DriverStation")
            return
        self.logger.info(f"Alliance color {alliance_color}")
        self.logger.info("Setting blue alliance perspective rotation")
        self.set_operator_perspective_forward(
            drivetrain.constants.RED_ALLIANCE_PERSPECTIVE_ROTATION
            if alliance_color == wpilib.DriverStation.Alliance.kRed
            else drivetrain.constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
        )

    def execute(self) -> None:
        pass

    def isManual(self):
        return True
