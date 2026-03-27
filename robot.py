import math

import magicbot
import phoenix6
import wpilib
import wpimath
from phoenix6 import swerve, hardware

import constants
from common import alliance, datalog, joystick
from subsystem import drivetrain, shooter, intake
from subsystem.drivetrain import limelight

DEADBAND = 0.15**2


class MyRobot(magicbot.MagicRobot):
    """Top-level robot class.

    This class uses the MagicBot framework to set up and manage the robot's
    subsystems and modes.
    https://robotpy.readthedocs.io/en/latest/frameworks/magicbot.html
    """

    intake_deployer: intake.IntakeDeployer
    shooter_state_machine: shooter.Shooter
    alliance_fetcher: alliance.AllianceFetcher

    target_tracker: shooter.TargetTracker
    drivetrain: drivetrain.Drivetrain
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    hood: shooter.Hood
    intake: intake.Intake
    turret: shooter.Turret
    vision: drivetrain.Vision
    rumble: joystick.DriverControllerRumble

    def createObjects(self) -> None:
        """Create and initialize robot objects."""
        self.robot_constants: constants.RobotConstants = (
            constants.get_robot_constants()
        )
        self.logger.info(f"Robot serial number: {self.robot_constants.serial}")

        self.driver_controller = joystick.DriverController(
            wpilib.XboxController(0),
            self.robot_constants.drivetrain.drive_options,
        )

        # Turret
        self.turret_motor = hardware.TalonFX(
            self.robot_constants.shooter.turret.motor_can_id,
            self.robot_constants.shooter.turret.motor_can_bus,
        )
        self.turret_encoder = hardware.CANcoder(
            self.robot_constants.shooter.turret.encoder_can_id,
            self.robot_constants.shooter.turret.encoder_can_bus,
        )

        # Flywheel motor and encoder.
        self.flywheel_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.flywheel.motor_can_id,
            self.robot_constants.shooter.flywheel.motor_can_bus,
        )
        self.flywheel_encoder = phoenix6.hardware.CANcoder(
            self.robot_constants.shooter.flywheel.encoder_can_id,
            self.robot_constants.shooter.flywheel.encoder_can_bus,
        )

        # Hopper motors.
        self.hopper_left_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.hopper.left_motor_can_id,
            self.robot_constants.shooter.hopper.left_motor_can_bus,
        )
        self.hopper_right_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.hopper.right_motor_can_id,
            self.robot_constants.shooter.hopper.right_motor_can_bus,
        )

        # Indexer motors.
        self.indexer_back_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.indexer.back_motor_can_id,
            self.robot_constants.shooter.indexer.back_motor_can_bus,
        )
        self.indexer_front_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.indexer.front_motor_can_id,
            self.robot_constants.shooter.indexer.front_motor_can_bus,
        )

        # Hood motor and encoder.
        self.hood_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.shooter.hood.motor_can_id,
            self.robot_constants.shooter.hood.motor_can_bus,
        )
        self.hood_encoder = phoenix6.hardware.CANcoder(
            self.robot_constants.shooter.hood.encoder_can_id,
            self.robot_constants.shooter.hood.encoder_can_bus,
        )

        # Intake motors.
        self.intake_roller_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.intake.roller_motor_can_id,
            self.robot_constants.intake.roller_motor_can_bus,
        )
        self.intake_deploy_motor = phoenix6.hardware.TalonFX(
            self.robot_constants.intake.deploy_motor_can_id,
            self.robot_constants.intake.deploy_motor_can_bus,
        )
        self.intake_deploy_encoder = phoenix6.hardware.CANcoder(
            self.robot_constants.intake.deploy_encoder_can_id,
            self.robot_constants.intake.deploy_encoder_can_bus,
        )

        self.data_logger = datalog.DataLogger()

        self._tuning_mode = False
        self._auto_done = False

    def robotPeriodic(self) -> None:
        if wpilib.DriverStation.isEnabled():
            # Deploy the intake.
            self.intake_deployer.deploy()
            # Always use external IMU to seed heading for the Limelights. This
            # will make localization worse when rotating fast, but it converges
            # much faster after rotation slows/stops.
            #
            # We were seeing very slow convergence on IMU mode 4.
            # TODO: Try playing with the alpha values.
            self.vision.setImuMode(1)
        else:
            # Hard reset each lime light's yaw to the external IMU when disabled.
            self.vision.setImuMode(1)
            # We call this here because the Vision component's execute method
            # does not get called when disabled.
            self.vision.setRobotOrientation()
            self.drivetrain._maybeSetOperatorPerspectiveForward()

        if not self._tuning_mode:
            self.shooter_state_machine.engage()
        else:
            # In tuning mode, have target tracker actively track, but don't command
            # the mechanisms.
            self.target_tracker.trackPosition(True)
            self.target_tracker.trackSpeed(True)
            self.target_tracker.setEnabled(False)

        super().robotPeriodic()

    def autonomousInit(self) -> None:
        """Initialize autonomous mode.

        This is called each time the robot enters autonomous mode, regardless of
        the selected autonomous routine.
        """
        self.logger.info("Entering autonomous mode")
        self.drivetrain.setAutoEnabled(True)
        self._auto_done = True

    def disabledInit(self) -> None:
        """Initialize disabled mode.

        This is called each time the robot enters disabled mode. The
        `on_disable` method of all components are called before this method is
        called.
        """
        self.logger.info("Robot disabled")
        self.drivetrain.setAutoEnabled(False)
        # This starts the log manager if it wasn't already started.
        self.data_logger.flush()
        self.driver_controller.setRumble(0.0)

    def disabledPeriodic(self) -> None:
        """Run during disabled mode.

        This is called periodically at a regular rate when the robot is in
        disabled mode. This code executes before the `execute` method of all
        components are called.
        """
        # Seed our pose estimator with the initial pose of the selected auto
        # mode, if we haven't run our auto yet.
        if not self._auto_done and self._automodes is not None:
            auto_mode = self._automodes.chooser.getSelected()
            if auto_mode is not None:
                self.drivetrain.setPose(auto_mode.getInitialPose())

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
        if self.driver_controller.resetOrientation():
            # TODO: Reset our pose from MT1 vision instead.
            if self.alliance_fetcher.isRedAlliance():
                # Robot's front touching the hub wall in the red alliance zone.
                self.drivetrain.setPose(
                    wpimath.geometry.Pose2d(12.8556, 4.0136, math.pi)
                )
                self.logger.info(f"Reset pose for red alliance")
            else:
                # Robot's front touching the hub wall in the blue alliance zone.
                self.drivetrain.setPose(
                    wpimath.geometry.Pose2d(3.6854, 4.0136, 0)
                )
                self.logger.info(f"Reset pose for blue alliance")
        self.driveWithJoysicks()
        self.controlShooter()
        self.controlIntake()

    def driveWithJoysicks(self) -> None:
        """Use the main controller joystick inputs to drive the robot base."""
        command = self.driver_controller.getDriveCommand()
        self.drivetrain.setSpeeds(command)
        self.drivetrain.setBrakeEnabled(self.driver_controller.brake())

    def controlShooter(self) -> None:
        """Takes button inputs to control the shooter state machine."""
        if self.driver_controller.shootFromLeftTrench():
            self.shooter_state_machine.setAuto(False)
            self.shooter_state_machine.setDriverWantsFeed(True)
            self.target_tracker.setTargetTurretAngleDegrees(4.268)
            self.target_tracker.setTargetHoodAngleDegrees(4.8)
            self.target_tracker.setTargetFlywheelSpeedRps(30.8)
        elif self.driver_controller.shootFromRightTrench():
            self.shooter_state_machine.setAuto(False)
            self.shooter_state_machine.setDriverWantsFeed(True)
            self.target_tracker.setTargetTurretAngleDegrees(-4.268)
            self.target_tracker.setTargetHoodAngleDegrees(4.8)
            self.target_tracker.setTargetFlywheelSpeedRps(30.8)
        elif self.driver_controller.shootFromBehindTower():
            self.shooter_state_machine.setAuto(False)
            self.shooter_state_machine.setDriverWantsFeed(True)
            self.target_tracker.setTargetTurretAngleDegrees(94.85)
            self.target_tracker.setTargetHoodAngleDegrees(6.9)
            self.target_tracker.setTargetFlywheelSpeedRps(33.8)
        else:
            self.shooter_state_machine.setAuto(True)
            if self.driver_controller.feedFuel():
                self.shooter_state_machine.setDriverWantsFeed(True)
            else:
                self.shooter_state_machine.setDriverWantsFeed(False)
                self.target_tracker.setTargetFlywheelSpeedRps(0.0)

    def controlIntake(self) -> None:
        """Drive the intake motors."""
        if self.driver_controller.toggleIntake():
            self.intake.toggleActive()
