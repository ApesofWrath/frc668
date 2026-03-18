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
BLUE_ALLIENCE_END_X = 5.05
RED_ALLIENCE_END_X = 11.5
FIELD_HEIGHT = 8.05

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
        self._target_hood_angle_degrees: float = 0.0
        self._target_flywheel_speed_rps: float = 0.0
        
        # Whether to re-calculate the flywheel speed each loop.
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

        if self._enabled:
            # TODO: make the shooter track the allience side (0/180deg) +/- 5 or 10deg depending on the pose y
            if(self._turret_field_pose.Y() > FIELD_HEIGHT/2):
                #do stuff
                pass
            else:
                pass
                #do the other stuff
            # self.turret.setPosition(self._target_turret_angle_degrees)
            # self.hood.setPosition(self._target_hood_angle_degrees)
            # self.flywheel.setTargetRps(self._target_flywheel_speed_rps)



