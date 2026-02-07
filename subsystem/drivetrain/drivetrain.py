import math
import wpimath
import constants

from phoenix6 import hardware
from wpilib import SmartDashboard
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import (
    SwerveDrive4Kinematics,
    ChassisSpeeds,
    SwerveModuleState,
)

import subsystem.drivetrain.swervemodule as swervemodule


class Drivetrain:

    def __init__(self):

        self.front_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FL,
            offset=constants.FL_OFFSET,
            turning_motor_id=constants.STEER_CAN_FL,
            turning_encoder_id=constants.TURN_ENCODER_ID_FL,
            name="Front Left",
        )

        self.front_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FR,
            offset=constants.FR_OFFSET,
            turning_motor_id=constants.STEER_CAN_FR,
            turning_encoder_id=constants.TURN_ENCODER_ID_FR,
            name="Front Right",
        )

        self.back_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BR,
            offset=constants.BR_OFFSET,
            turning_motor_id=constants.STEER_CAN_BR,
            turning_encoder_id=constants.TURN_ENCODER_ID_BR,
            name="Back Right",
        )

        self.back_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BL,
            offset=constants.BL_OFFSET,
            turning_motor_id=constants.STEER_CAN_BL,
            turning_encoder_id=constants.TURN_ENCODER_ID_BL,
            name="Back Left",
        )

        self.gyro = hardware.Pigeon2(22, "rio")

        self.kinematics = SwerveDrive4Kinematics(
            Translation2d(
                constants.WHEEL_BASE / 2, constants.TRACK_WIDTH / 2
            ),  # Front Left
            Translation2d(
                constants.WHEEL_BASE / 2, -constants.TRACK_WIDTH / 2
            ),  # Front Right
            Translation2d(
                -constants.WHEEL_BASE / 2, constants.TRACK_WIDTH / 2
            ),  # Back Left
            Translation2d(
                -constants.WHEEL_BASE / 2, -constants.TRACK_WIDTH / 2
            ),  # Back Right
        )

        self.vx = 0
        self.vy = 0
        self.omega = 0
        self.gyro.set_yaw(0)

        self.front_left.resetDriveMotorPosition()
        self.front_right.resetDriveMotorPosition()
        self.back_left.resetDriveMotorPosition()
        self.back_right.resetDriveMotorPosition()

    def drive(self, speeds: ChassisSpeeds, field_relative: bool = True) -> None:
        """
        Sets the desired state for each module based on the givven ChassisSpeeds.

        Params:
            speeds (ChassisSpeeds): Target velocities for the bot.
            field_relative (bool): Whether the bot should be field relative or not
        """

        if field_relative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vx, speeds.vy, speeds.omega, self.gyro.getRotation2d()
            )

        states = self.kinematics.toSwerveModuleStates(speeds)
        SwerveDrive4Kinematics.desaturateWheelSpeeds(
            states, constants.MAX_LINEAR_SPEED
        )

        self.front_left.setDesiredState(states[0])
        self.front_right.setDesiredState(states[1])
        self.back_left.setDesiredState(states[2])
        self.back_right.setDesiredState(states[3])

    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the drivetrain based off current state.
        """

        speeds = ChassisSpeeds(self.vx, self.vy, self.omega)
        self.drive(speeds, field_relative=True)

        SmartDashboard.putNumber(
            "FL Encoder", self.front_left.getEncoderAngleDeg()
        )
        SmartDashboard.putNumber(
            "FR Encoder", self.front_right.getEncoderAngleDeg()
        )
        SmartDashboard.putNumber(
            "BL Encoder", self.back_left.getEncoderAngleDeg()
        )
        SmartDashboard.putNumber(
            "BR Encoder", self.back_right.getEncoderAngleDeg()
        )

    def isManual(self):
        """
        Returns whether the robot is being controlled by the driver

        Currently just returns True, but could return false when doing precise alignments
        """
        return True
