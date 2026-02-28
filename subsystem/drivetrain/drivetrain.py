import math

import wpilib
import wpimath
from wpimath.geometry import Pose2d
# from wpimath.kinematics import ChassisSpeeds
from phoenix6 import hardware, swerve
from phoenix6.swerve.requests import ApplyRobotSpeeds

from subsystem.drivetrain import constants


class Drivetrain(swerve.SwerveDrivetrain):
    def __init__(self):
        super().__init__(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            constants.TunerConstants.drivetrain_constants,
            [
                constants.TunerConstants.front_left,
                constants.TunerConstants.front_right,
                constants.TunerConstants.back_left,
                constants.TunerConstants.back_right,
            ],
        )

    def setup(self) -> None:
        """Apply the operator perspective based on alliance color."""
        alliance_color = wpilib.DriverStation.getAlliance()
        if alliance_color is None:
            self.logger.error("Failed to get alliance from DriverStation")
            return
        self.logger.info(f"Alliance color {alliance_color}")
        self.set_operator_perspective_forward(
            constants.RED_ALLIANCE_PERSPECTIVE_ROTATION
            if alliance_color == wpilib.DriverStation.Alliance.kRed
            else constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
        )

    def execute(self) -> None:
        pass

    def isManual(self):
        return True
    
    # def get_pose(self) -> Pose2d:
    #     return self.get_state().pose

    def reset_odometry(self, pose: Pose2d):
        self.seed_field_centric(pose.rotation())

    # def get_speeds(self) -> ChassisSpeeds:
    #     return self.get_state().speeds
    
    # def drive_robot_relative(self, speeds: ChassisSpeeds, feedforwards: DriveFeedforwards):
    #     req = ApplyRobotSpeeds()
    #     req.speeds = speeds
    #     req.
    #     self.set_control(req)
    