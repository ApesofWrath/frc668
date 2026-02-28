import magicbot
import wpilib
from phoenix6 import swerve

import constants
from common import joystick
from subsystem import drivetrain


class TestChassis(magicbot.MagicRobot):
    def createObjects(self) -> None:
        self.robot_constants: constants.RobotConstants = (
            constants.get_robot_constants()
        )
        self.logger.info(
            f"Using constants for serial #{self.robot_constants.serial}"
        )

        self.drivetrain: drivetrain.Drivetrain = drivetrain.Drivetrain(
            self.robot_constants.drivetrain
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

        self.drivetrain.setup()

    def robotInit(self) -> None:
        """MagicBot internal API

        Do NOT add anything in here!
        """
        super().robotInit()

        # Technically, we shouldn't be overriding this method. But we need to
        # add our Drivetrain component to magicbot's internal list so its
        # on_enable, on_disable, and execute methods are called appropriately.
        self._components.append(("drivetrain", self.drivetrain))

    def disabledPeriodic(self) -> None:
        # Periodically try to set operator perspective, in case we weren't able
        # to during setup.
        self.drivetrain.maybeSetOperatorPerspectiveForward()

    def teleopPeriodic(self) -> None:
        if self.joystick.should_reset_orientation():
            self.drivetrain.seed_field_centric()

        command = self.joystick.get_drive_command()
        self.drivetrain.setSpeeds(command)
