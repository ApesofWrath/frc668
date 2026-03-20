import magicbot
import wpilib
from phoenix6 import swerve

import constants
from common import joystick
from subsystem import drivetrain


class TestChassis(magicbot.MagicRobot):
    drivetrain: drivetrain.Drivetrain

    def createObjects(self) -> None:
        self.robot_constants: constants.RobotConstants = (
            constants.get_robot_constants()
        )
        self.logger.info(
            f"Using constants for serial #{self.robot_constants.serial}"
        )

        self.joystick: joystick.DriverJoystick = joystick.DriverController(
            wpilib.XboxController(0),
            self.robot_constants.drivetrain.drive_options,
        )
        self.drive_request = (
            swerve.requests.FieldCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE
            )
        )

    def disabledPeriodic(self) -> None:
        # Periodically try to set operator perspective, in case we weren't able
        # to during setup.
        self.drivetrain._maybeSetOperatorPerspectiveForward()

    def teleopPeriodic(self) -> None:
        if self.joystick.resetOrientation():
            self.drivetrain.seed_field_centric()

        command = self.joystick.getDriveCommand()
        self.drivetrain.setSpeeds(command)
