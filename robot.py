import math

import magicbot
import phoenix6
import wpilib
import wpimath
from phoenix6 import swerve, hardware

import constants
from common import joystick
from subsystem import drivetrain, shooter, intake
from subsystem.drivetrain import limelight

DEADBAND = 0.15**2


class MyRobot(magicbot.MagicRobot):
    """Top-level robot class.

    This class uses the MagicBot framework to set up and manage the robot's
    subsystems and modes.
    https://robotpy.readthedocs.io/en/latest/frameworks/magicbot.html
    """

    drivetrain: drivetrain.Drivetrain
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    hood: shooter.Hood
    intake: intake.Intake
    turret: shooter.Turret
    vision: drivetrain.Vision

    def createObjects(self) -> None:
        """Create and initialize robot objects."""
        self.robot_constants: constants.RobotConstants = (
            constants.get_robot_constants()
        )

        # We instantiate the Drivetrain component manually, because magicbot
        # does not allow accessing injected variables from component
        # constructors. Our Drivetrain component needs to access robot constants
        # in the constructor to properly initialize its parent class
        # (phoenix6.swerve.SwerveDrivetrain). Therefore we cannot rely on
        # self.robot_constants being injected automatically into Drivetrain, and
        # must pass the constants into the constructor manually.
        #
        # Since we manually instantiate drivetrain, magicbot will not attempt to
        # auto-instantiate it for us. Keeping the annotation above is helpful so
        # that other components can benefit from automatic injection of
        # Drivetrain, and other benefits like IDE autocomplete/type checking.
        self.drivetrain = drivetrain.Drivetrain(self.robot_constants.drivetrain)

        self.driver_controller = joystick.DriverController(
            wpilib.XboxController(0),
            self.robot_constants.drivetrain.drive_options,
        )
        self.operator_controller = wpilib.XboxController(1)

        # Turret
        self.turret_motor = hardware.TalonFX(
            self.robot_constants.shooter.turret.motor_can_id, "Shooter"
        )
        self.turret_encoder = hardware.CANcoder(
            self.robot_constants.shooter.turret.encoder_can_id, "Shooter"
        )

        # Flywheel motor and encoder.
        self.flywheel_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.flywheel.motor_can_id, "Shooter"
        )
        self.flywheel_encoder = phoenix6.hardware.CANcoder(
            self.robot_constants.shooter.flywheel.encoder_can_id, "Shooter"
        )

        # Hopper motors.
        self.hopper_left_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.hopper.left_motor_can_id, "rio"
        )
        self.hopper_right_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.hopper.right_motor_can_id, "rio"
        )

        # Indexer motors.
        self.indexer_back_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.indexer.back_motor_can_id, "Shooter"
        )
        self.indexer_front_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.indexer.front_motor_can_id, "Shooter"
        )

        # Hood motor and encoder.
        self.hood_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.hood.motor_can_id, "Shooter"
        )
        self.hood_encoder = phoenix6.hardware.CANcoder(
            self.robot_constants.shooter.hood.encoder_can_id, "Shooter"
        )

        # Intake motor.
        self.intake_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.intake.motor_can_id, "rio"
        )

        self._tuning_mode = False

        # Since we manually instantiate Drivetrain, magicbot will not call setup
        # for us.
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
        # And we need to register its feedback methods.
        self._feedbacks += magicbot.magic_tunable.collect_feedbacks(
            self.drivetrain, "drivetrain", "components"
        )

    def robotPeriodic(self) -> None:
        if wpilib.DriverStation.isEnabled():
            # Use external IMU assist when enabled.
            for ll in self.vision._limelights:
                limelight.LimelightHelpers.set_imu_mode(ll, 4)
        else:
            # Hard reset each limelight's yaw to the external IMU when disabled.
            for ll in self.vision._limelights:
                limelight.LimelightHelpers.set_imu_mode(ll, 1)
                # We call this here because the Vision component's execute
                # method does not get called when disabled.
                self.vision.setRobotOrientation()

    def autonomousInit(self) -> None:
        """Initialize autonomous mode.

        This is called each time the robot enters autonomous mode, regardless of
        the selected autonomous routine.
        """
        self.logger.info("Entering autonomous mode")

    def disabledInit(self) -> None:
        """Initialize disabled mode.

        This is called each time the robot enters disabled mode. The
        `on_disable` method of all components are called before this method is
        called.
        """
        self.logger.info("Robot disabled")
        self.vision._pose_seeded = False
        self.vision.imu_four = False

    def disabledPeriodic(self) -> None:
        """Run during disabled mode.

        This is called periodically at a regular rate when the robot is in
        disabled mode. This code executes before the `execute` method of all
        components are called.
        """
        for ll in self.vision._limelights:
            limelight.LimelightHelpers.set_imu_mode(ll, 1)
            self.vision.setRobotOrientation()

        # Periodically try to set operator perspective, in case we weren't able
        # to during setup.
        self.drivetrain.maybeSetOperatorPerspectiveForward()
        # We dont want to be zeroing the turret while it's moving, so we'll zero
        # it while its disabled
        if self.operator_controller.getStartButton():
            self.turret.zeroEncoder()

    def teleopInit(self) -> None:
        """Initialize teleoperated mode.

        This is called each time the robot enters teleoperated mode. The
        `on_enable` method of all components are called before this method is
        called.
        """
        self.logger.info("Entering teleop mode")

    def teleopPeriodic(self) -> None:
        """Run during teleoperated mode.

        This is called periodically at a regular rate when the robot is in
        teleoperated mode. This code executes before the `execute` method of all
        components are called.

        If you want this method to be called in autonomous mode, set
        `use_teleop_in_autonomous=True` in this class' instance.
        """
        # TODO: Handle exceptions so robot code doesn't crash.
        if self.driver_controller.shouldResetOrientation():
            self.drivetrain.reset_pose(wpimath.geometry.Pose2d(0, 0, 0))
            self.vision._pose_seeded = False
        self.driveWithJoysicks()
        self.controlHopper()
        self.controlIndexer()
        self.controlIntake()
        self.controlHood()
        self.controlTurret()

    def driveWithJoysicks(self) -> None:
        """Use the main controller joystick inputs to drive the robot base."""
        command = self.driver_controller.getDriveCommand()
        self.drivetrain.setSpeeds(command)

    def controlHopper(self) -> None:
        """Drive the hopper motors."""
        if self._tuning_mode:
            return
        if self.operator_controller.getRightBumper():
            self.hopper.setEnabled(True)
        else:
            self.hopper.setEnabled(False)

    def controlIndexer(self) -> None:
        """Drive the indexer motors."""
        if self._tuning_mode:
            return
        if self.operator_controller.getRightBumper():
            self.indexer.setEnabled(True)
        else:
            self.indexer.setEnabled(False)

    def controlIntake(self) -> None:
        """Drive the intake motors."""
        if self.driver_controller.runIntake():
            self.intake.setMotorSpeed(1.0)
        else:
            self.intake.setMotorSpeed(0.0)

    def controlHood(self) -> None:
        """Drive the hood motor."""
        # Zero the hood encoder if the B button was pressed.
        if self.operator_controller.getBButtonPressed():
            self.hood.zeroEncoder()

        # Toggle between manual hood speed control v/s position control.
        if self.operator_controller.getYButtonReleased():
            self.hood.setControlType(not self.hood.isControlTypeSpeed())
            self.logger.info(
                "Hood control type is now: "
                + ("speed" if self.hood.isControlTypeSpeed() else "position")
            )

        # Drive the hood motor at one-fourth duty cycle.
        if self.hood.isControlTypeSpeed():
            self.hood.setSpeed(
                -filterInput(self.operator_controller.getRightY()) * 0.1
            )

    def controlTurret(self) -> None:
        """Drive the turret motor."""
        if self.operator_controller.getXButtonReleased():
            self.turret.setControlType(not self.turret.isControlTypeVelocity())
            self.logger.info(
                "Turret control type is now: "
                + (
                    "velocity"
                    if self.turret.isControlTypeVelocity()
                    else "position"
                )
            )
        if self.turret.isControlTypeVelocity():
            self.turret.setVelocity(
                filterInput(self.operator_controller.getLeftX()) * 30
            )


def filterInput(controller_input: float, apply_deadband: bool = True) -> float:
    """Filter the controller input with a squared scaling and deadband.

    This function squares the input while preserving its sign to provide finer
    control at lower values. If `apply_deadband` is True, it applies a deadband
    to ignore small inputs that may result from controller drift.

    Args:
        controller_input:
            The raw input from the controller, ranging from -1.0 to 1.0.
        apply_deadband:
            Whether to apply a deadband to the input. Defaults to True.

    Returns:
        A float that is the filtered controller input.
    """
    controller_input_corrected = math.copysign(
        math.pow(controller_input, 2), controller_input
    )

    if apply_deadband:
        return wpimath.applyDeadband(controller_input_corrected, DEADBAND)
    else:
        return controller_input_corrected
