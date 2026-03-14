import math

import magicbot
import phoenix6
import wpilib
import wpimath
from phoenix6 import swerve, hardware

from magicbot import MagicRobot
from wpilib import DriverStation, SmartDashboard

import constants
from common import alliance, joystick
from subsystem import drivetrain, shooter, intake
from autonomous import AutoHelper
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

    hub_tracker: shooter.HubTracker
    drivetrain: drivetrain.Drivetrain
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    hood: shooter.Hood
    intake: intake.Intake
    turret: shooter.Turret
    vision: drivetrain.Vision

    AutoHelper: AutoHelper.AutoHelper


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

        self._tuning_mode = False

    def robotPeriodic(self) -> None:
        if wpilib.DriverStation.isEnabled():
            if not self.intake_deployer._deployed:
                self.intake_deployer.deploy()
            for ll in self.vision._limelights:
                limelight.LimelightHelpers.set_imu_mode(ll, 4)
        else:
            for ll in self.vision._limelights:
                limelight.LimelightHelpers.set_imu_mode(ll, 1)
                self.vision.setRobotOrientation()

        if not self._tuning_mode:
            self.shooter_state_machine.engage()
        else:
            # In tuning mode, have hub tracker actively track, but don't command
            # the mechanisms.
            self.hub_tracker.trackPosition(True)
            self.hub_tracker.trackSpeed(True)
            self.hub_tracker.setEnabled(False)

        # Required for SmartDashboard choosers to work (without this, you cant change autos)
        super().robotPeriodic()

    def autonomousInit(self) -> None:
        """Initialize autonomous mode.

        This is called each time the robot enters autonomous mode, regardless of
        the selected autonomous routine.
        """

        self.logger.info("Entering autonomous mode")

    def disabledPeriodic(self) -> None:
        """Run during disabled mode.

        This is called periodically at a regular rate when the robot is in
        disabled mode. This code executes before the `execute` method of all
        components are called.
        """
        for ll in self.vision._limelights:
            limelight.LimelightHelpers.set_imu_mode(ll, 1)
            self.vision.setRobotOrientation()
        self.drivetrain._maybeSetOperatorPerspectiveForward()
        # self.logger.info(self._automodes.chooser.getSelected())

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
            alliance = self.alliance_fetcher.getAlliance()
            if alliance == wpilib.DriverStation.Alliance.kRed:
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
            self.vision._pose_seeded = False
        self.driveWithJoysicks()
        self.controlShooter()
        self.controlIntake()

    def driveWithJoysicks(self) -> None:
        """Use the main controller joystick inputs to drive the robot base."""
        command = self.driver_controller.getDriveCommand()
        self.drivetrain.setSpeeds(command)

    def controlShooter(self) -> None:
        """Takes button inputs to control the shooter state machine."""
        if self.driver_controller.setLeftFieldDefaults():
            self.shooter_state_machine.setAuto(False)
            self.shooter_state_machine.setDriverWantsFeed(True)
            self.hub_tracker.setTargetTurretAngleDegrees(4.268)
            self.hub_tracker.setTargetHoodAngleDegrees(4.8)
            self.hub_tracker.setTargetFlywheelSpeedRps(30.8)
        elif self.driver_controller.setRightFieldDefaults():
            self.shooter_state_machine.setAuto(False)
            self.shooter_state_machine.setDriverWantsFeed(True)
            self.hub_tracker.setTargetTurretAngleDegrees(-4.268)
            self.hub_tracker.setTargetHoodAngleDegrees(4.8)
            self.hub_tracker.setTargetFlywheelSpeedRps(30.8)
        elif self.driver_controller.setCenterFieldDefaults():
            self.shooter_state_machine.setAuto(False)
            self.shooter_state_machine.setDriverWantsFeed(True)
            self.hub_tracker.setTargetTurretAngleDegrees(94.85)
            self.hub_tracker.setTargetHoodAngleDegrees(6.9)
            self.hub_tracker.setTargetFlywheelSpeedRps(33.8)
        else:
            self.shooter_state_machine.setAuto(True)
            if self.driver_controller.feedFuel():
                self.shooter_state_machine.setDriverWantsFeed(True)
            else:
                self.shooter_state_machine.setDriverWantsFeed(False)
                self.hub_tracker.setTargetFlywheelSpeedRps(0.0)

    def controlIntake(self) -> None:
        """Drive the intake motors."""
        if self.driver_controller.toggleIntake():
            self.intake.toggleActive()
