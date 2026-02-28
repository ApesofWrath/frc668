import typing
from dataclasses import dataclass

import wpimath
from phoenix6 import configs, signals, swerve, units


# These constants are typically common across the swerve modules.
# https://api.ctr-electronics.com/phoenix6/stable/python/autoapi/phoenix6/swerve/swerve_module_constants/index.html#phoenix6.swerve.swerve_module_constants.SwerveModuleConstantsFactory
@dataclass(frozen=True)
class SwerveModuleCommonConstants:
    # Gear ratio between the drive motor and the wheel.
    drive_motor_gear_ratio: float = 4.67
    # Gear ratio betwee nthe steer motor and the azimuth encoder.
    steer_motor_gear_ratio: float = 25.9
    # Coupled gear ratio between the azimuth encoder and the drive motor.
    coupling_gear_ratio: float = 0.0
    # Radius of the driving wheel in meters.
    wheel_radius: units.meter = 0.04445
    # Steer motor closed-loop gains. These gains operate on azimuth rotations,
    # *after* the gear ratio.
    steer_motor_gains: configs.Slot0Configs = configs.Slot0Configs()
    # Drive motor closed-loop gains. These gains operate on rotor rotations,
    # *before* the gear ratio.
    drive_motor_gains: configs.Slot0Configs = configs.Slot0Configs()
    # The closed-loop output type to use for the steer motors.
    steer_motor_closed_loop_output: swerve.ClosedLoopOutputType = (
        swerve.ClosedLoopOutputType.VOLTAGE
    )
    # The closed-loop output type to use for the drive motors.
    drive_motor_closed_loop_output: swerve.ClosedLoopOutputType = (
        swerve.ClosedLoopOutputType.VOLTAGE
    )
    # The maximum amount of stator current the drive motors can apply without
    # slippage.
    slip_current: units.ampere = 120.0
    # When using open-loop drive control, this specifies he speed at which the
    # robot travels when driven with 12 volts. This is used to approximate the
    # output for a desired velocity. If using closed loop output, this value is
    # ignored.
    speed_at12_volts: units.meters_per_second = 12.0
    # Motor used for drive.
    drive_motor_type: swerve.DriveMotorArrangement = (
        swerve.DriveMotorArrangement.TALON_FX_INTEGRATED
    )
    # Motor used for steer.
    steer_motor_type: swerve.SteerMotorArrangement = (
        swerve.SteerMotorArrangement.TALON_FX_INTEGRATED
    )
    # How the feedback sensors should be configured.
    feedback_source: swerve.SteerFeedbackType = (
        swerve.SteerFeedbackType.FUSED_CANCODER
    )
    # The initial configs used to configure the drive motor of the swerve
    # module.
    drive_motor_initial_configs: configs.TalonFXConfiguration = (
        configs.TalonFXConfiguration()
    )
    # The initial configs used to configure the steer motor of the swerve
    # module.
    steer_motor_initial_configs: (
        configs.TalonFXConfiguration
    ) = configs.TalonFXConfiguration().with_current_limits(
        configs.CurrentLimitsConfigs()
        .with_stator_current_limit(60.0)
        .with_stator_current_limit_enable(True)
    )
    # The initial configs used to configure the azimuth encoder of the swerve
    # module.
    encoder_initial_configs: configs.CANcoderConfiguration = (
        configs.CANcoderConfiguration()
    )


# These constants are typically unique for each swerve module.
@dataclass(frozen=True)
class SwerveModuleConstants:
    # CAN ID of the steer motor.
    steer_motor_id: int = 0
    # CAN ID of the drive motor.
    drive_motor_id: int = 0
    # CAN ID of the absolute encoder used for azimuth.
    encoder_id: int = 0
    # Offset of the azimuth encoder.
    encoder_offset: units.rotation = 0.0
    # The location of this module's wheels relative to the physical center of
    # the robot in meters along the X axis of the robot.
    location_x: units.meter = 0
    # The location of this module's wheels relative to the physical center of
    # the robot in meters along the Y axis of the robot.
    location_y: units.meter = 0
    # True if the drive motor is inverted.
    drive_motor_inverted: bool = False
    # True if the steer motor is inverted from the azimuth. The azimuth should
    # rotate counter-clockwise (as seen from the top of the robot) for a
    # positive motor output.
    steer_motor_inverted: bool = False
    # True if the azimuth encoder is inverted from the azimuth. The encoder
    # should report a positive velocity when the azimuth rotates
    # counter-clockwise (as seen from the top of the robot).
    encoder_inverted: bool = False


# Common constants for a swerve drivetrain.
@dataclass(frozen=True)
class SwerveDrivetrainConstants:
    can_bus_name: str = ""
    pigeon2_id: int = 0
    pigeon2_configs: typing.Optional[configs.Pigeon2Configuration] = None


# Options related to drivetrain limits.
@dataclass(frozen=True)
class DriveOptions:
    max_linear_speed_meters_per_second: float = 6.0
    max_linear_acceleration_meters_per_second_squared: float = 3.0
    max_angular_speed_radians_per_second: float = 6.0
    max_angular_acceleration_radians_per_second_squared: float = 0.5


# Collection of all drivetrain-related constants for the robot.
@dataclass(frozen=True)
class DrivetrainConstants:
    common: SwerveModuleCommonConstants = SwerveModuleCommonConstants()
    front_left: SwerveModuleConstants = SwerveModuleConstants()
    front_right: SwerveModuleConstants = SwerveModuleConstants()
    back_left: SwerveModuleConstants = SwerveModuleConstants()
    back_right: SwerveModuleConstants = SwerveModuleConstants()
    drivetrain: SwerveDrivetrainConstants = SwerveDrivetrainConstants()
    drive_options: DriveOptions = DriveOptions()


# Constants per robot serial number.
DRIVETRAIN_CONSTANTS: dict[str, DrivetrainConstants] = {
    # Test chassis
    "0323800E": DrivetrainConstants(
        common=SwerveModuleCommonConstants(
            steer_motor_gains=configs.Slot0Configs()
            .with_k_s(0.1)
            .with_k_v(2.48)
            .with_k_p(100)
            .with_k_d(0.5)
            .with_static_feedforward_sign(
                signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            ),
            drive_motor_gains=configs.Slot0Configs()
            .with_k_v(0.124)
            .with_k_p(0.1),
        ),
        front_left=SwerveModuleConstants(
            steer_motor_id=8,
            drive_motor_id=7,
            encoder_id=12,
            encoder_offset=130 / 360,
            location_x=wpimath.units.inchesToMeters(13.75),
            location_y=wpimath.units.inchesToMeters(13.75),
            drive_motor_inverted=True,
            steer_motor_inverted=False,
            encoder_inverted=True,
        ),
        front_right=SwerveModuleConstants(
            steer_motor_id=4,
            drive_motor_id=3,
            encoder_id=10,
            encoder_offset=-5 / 360,
            location_x=wpimath.units.inchesToMeters(13.75),
            location_y=wpimath.units.inchesToMeters(-13.75),
            drive_motor_inverted=False,
            steer_motor_inverted=False,
            encoder_inverted=True,
        ),
        back_left=SwerveModuleConstants(
            steer_motor_id=2,
            drive_motor_id=1,
            encoder_id=9,
            encoder_offset=55 / 360,
            location_x=wpimath.units.inchesToMeters(-13.75),
            location_y=wpimath.units.inchesToMeters(13.75),
            drive_motor_inverted=True,
            steer_motor_inverted=False,
            encoder_inverted=False,
        ),
        back_right=SwerveModuleConstants(
            steer_motor_id=6,
            drive_motor_id=5,
            encoder_id=11,
            encoder_offset=250 / 360,
            location_x=wpimath.units.inchesToMeters(-13.75),
            location_y=wpimath.units.inchesToMeters(-13.75),
            drive_motor_inverted=False,
            steer_motor_inverted=False,
            encoder_inverted=False,
        ),
        drivetrain=SwerveDrivetrainConstants(pigeon2_id=22),
    ),
    # Alphabot
    "023AC96C": DrivetrainConstants(
        common=SwerveModuleCommonConstants(
            # TODO: Tune.
            steer_motor_gains=configs.Slot0Configs()
            .with_k_p(100)
            .with_static_feedforward_sign(
                signals.StaticFeedforwardSignValue.USE_CLOSED_LOOP_SIGN
            ),
            drive_motor_gains=configs.Slot0Configs()
            .with_k_v(0.124)
            .with_k_p(0.1),
        ),
        front_left=SwerveModuleConstants(
            steer_motor_id=8,
            drive_motor_id=7,
            encoder_id=12,
            encoder_offset=0.398926,
            location_x=wpimath.units.inchesToMeters(13.75),
            location_y=wpimath.units.inchesToMeters(13.75),
            drive_motor_inverted=False,
            steer_motor_inverted=False,
            encoder_inverted=True,
        ),
        front_right=SwerveModuleConstants(
            steer_motor_id=4,
            drive_motor_id=3,
            encoder_id=10,
            encoder_offset=0.434814,
            location_x=wpimath.units.inchesToMeters(13.75),
            location_y=wpimath.units.inchesToMeters(-13.75),
            drive_motor_inverted=True,
            steer_motor_inverted=False,
            encoder_inverted=True,
        ),
        back_left=SwerveModuleConstants(
            steer_motor_id=2,
            drive_motor_id=1,
            encoder_id=9,
            encoder_offset=-0.05835,
            location_x=wpimath.units.inchesToMeters(-13.75),
            location_y=wpimath.units.inchesToMeters(13.75),
            drive_motor_inverted=False,
            steer_motor_inverted=False,
            encoder_inverted=True,
        ),
        back_right=SwerveModuleConstants(
            steer_motor_id=6,
            drive_motor_id=5,
            encoder_id=11,
            encoder_offset=0.38794,
            location_x=wpimath.units.inchesToMeters(-13.75),
            location_y=wpimath.units.inchesToMeters(-13.75),
            drive_motor_inverted=True,
            steer_motor_inverted=False,
            encoder_inverted=True,
        ),
        drivetrain=SwerveDrivetrainConstants(pigeon2_id=22),
    ),
}


MAX_LINEAR_SPEED = 6  # meters per second
MAX_LINEAR_ACCELERATION = 3  # meters per second squared

MAX_ROTATION_SPEED = 6  # radians per second
MAX_ROTATION_ACCELERATION = 1 / 2  # radians per second squared

MAX_SINGLE_SWERVE_ROTATION_SPEED = 12  # radians per second
MAX_SINGLE_SWERVE_ROTATION_ACCELERATION = 40  # radians per sec squared

# Blue alliance sees forward as 0 degrees (toward red alliance wall)
BLUE_ALLIANCE_PERSPECTIVE_ROTATION = wpimath.geometry.Rotation2d.fromDegrees(0)
# Red alliance sees forward as 180 degrees (toward blue alliance wall)
RED_ALLIANCE_PERSPECTIVE_ROTATION = wpimath.geometry.Rotation2d.fromDegrees(180)
