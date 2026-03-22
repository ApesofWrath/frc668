import bisect
from typing import Tuple

import magicbot
import phoenix6
import wpilib
from wpimath import geometry, units

import constants
from common import alliance
from subsystem import drivetrain, shooter

#In meters.
BLUE_ALLIENCE_START_X = 0
BLUE_ALLIENCE_END_X = 5.05
RED_ALLIENCE_START_X = 16.5
RED_ALLIENCE_END_X = 11.5
FIELD_HEIGHT = 8.05

SHOOT_BACK_HOOD_ANGLE_DEG = 40
SHOOT_BACK_FLYWHEEL_RPS = 20 #TODO: find what this

class AllienceTracker:
    """For tracking our allience zone"""
    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain
    alliance_fetcher: alliance.AllianceFetcher
    flywheel: shooter.Flywheel
    hood: shooter.Hood
    turret: shooter.Turret
    
    def setup(self) -> None:
        # Pose of the turret relative to the field. This will be computed each
        # control loop based on the current robot pose estimate.
        self._turret_field_pose: geometry.Pose2d = geometry.Pose2d()

        self._alliance = self.alliance_fetcher.getAlliance() 

        # Current targets.
        # TODO: Find better defaults.
        self._target_turret_angle_degrees: float = 0.0
        self._target_hood_angle_degrees: float = SHOOT_BACK_HOOD_ANGLE_DEG
        self._target_flywheel_speed_rps: float = SHOOT_BACK_FLYWHEEL_RPS
        
        # Whether to reset the flywheel speed each loop.
        self._track_speed = False
        # Whether to command mechanisms to the current targets.
        self._enabled = True

    def execute(self) -> None:
        # Pose of the robot relative to field origin.
        robot_pose: geometry.Pose2d = (
            self.drivetrain.swerve_drive.get_state().pose
        )
        # An approxamation of the turret position, it doesn't really matter that much if it's off a bit
        self._turret_field_pose: geometry.Pose2d = robot_pose

        in_neutral_zone = (self._turret_field_pose.X() >= BLUE_ALLIENCE_START_X 
                          and self._turret_field_pose.X() <= RED_ALLIENCE_START_X)
        in_enemy_zone = (self._turret_field_pose.X() < BLUE_ALLIENCE_START_X
                         if self.alliance_fetcher.getAlliance() == wpilib.DriverStation.Alliance.kRed else
                         self._turret_field_pose.X() > RED_ALLIENCE_START_X)

        if self._enabled and (in_neutral_zone or in_enemy_zone):
            if(self._turret_field_pose.Y() > FIELD_HEIGHT/2): #TODO flip for both alliances
                self._target_turret_angle_degrees = self._computeTargetTurretAngleDegrees(-10) #TODO: check that this is the right way (it should be)
            else:
                self._target_turret_angle_degrees = self._computeTargetTurretAngleDegrees(10)
            self.turret.setPosition(self._target_turret_angle_degrees)
            self.hood.setPosition(self._target_hood_angle_degrees)
            if self._track_speed:
                self.flywheel.setTargetRps(self._target_flywheel_speed_rps)
        else:
            if self._track_speed:
                self.flywheel.setTargetRps(self.robot_constants.shooter.flywheel.default_speed_rps)


    def _computeTargetTurretAngleDegrees(
        self, offset_degrees: phoenix6.units.degree
    ) -> phoenix6.units.degree:
        """Returns the target angle of the turret."""
        alliance_pos = (
            geometry.Translation2d(RED_ALLIENCE_START_X 
                                   if self.alliance_fetcher.getAlliance() == wpilib.DriverStation.Alliance.kRed else 
                                   BLUE_ALLIENCE_START_X, 
                                   self._turret_field_pose.Y())
        )

        # Vector from center of turret to the alllience.
        turret_to_allience = (
            alliance_pos - self._turret_field_pose.translation()
        )
        target_angle_degrees = (
            turret_to_allience.angle() - self._turret_field_pose.rotation()
        ).degrees() + offset_degrees #TODO swap offset to just aim at the corner

        return max(
            self.robot_constants.shooter.turret.min_angle,
            min(self.robot_constants.shooter.turret.max_angle,target_angle_degrees),
        )
    
    def getInAllienceZone(self) -> bool:
        return (self._turret_field_pose.X() > BLUE_ALLIENCE_START_X
                if self.alliance_fetcher.getAlliance() == wpilib.DriverStation.Alliance.kBlue else
                self._turret_field_pose.X() < RED_ALLIENCE_START_X)

    def setEnabled(self, enabled):
        self._enabled = enabled

    def setTrack(self, speed, position):
        self._track_speed = speed
        self._enabled = position

    # @magicbot.feedback
    def get_target_turret_angle_degrees(self) -> phoenix6.units.degree:
        """Returns the target angle for the turret to track."""
        return self._target_turret_angle_degrees

    # @magicbot.feedback
    def get_target_hood_angle_degrees(self) -> phoenix6.units.degree:
        """Returns the target angle for the hood to track."""
        return self._target_hood_angle_degrees

    # @magicbot.feedback
    def get_target_flywheel_speed_rps(
        self,
    ) -> phoenix6.units.rotations_per_second:
        """Returns the target rps for the flywheel to track."""
        return self._target_flywheel_speed_rps