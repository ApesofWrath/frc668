import math

import wpimath
from magicbot import feedback, tunable
from phoenix6 import utils

import constants
from subsystem import drivetrain
from subsystem.drivetrain import limelight

from ntcore import NetworkTableInstance

import wpilib

RADIANS_TO_DEGREES = 180.0 / math.pi


class Vision:
    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain

    def setup(self) -> None:
        """
        Sets up varibles and has a list with limelight names,
        need to manually add more with more limelights.
        """
        self._limelights: list[str] = (
            self.robot_constants.drivetrain.vision.limelights
        )
        # Tracks whether the drivetrain's pose estimator has been seeded with a
        # good vision estimate since startup.
        self._pose_seeded = False

        self.xy_std_devs = 0.0
        self.theta_std_devs = 0.0

    def execute(self) -> None:
        self.setRobotOrientation()
        # if self.drivetrain.swerve_drive.pigeon2.get_pitch().value > 0.0:
        #     self.drivetrain.swerve_drive.set_state_std_devs(math.inf, math.inf, math.inf)
        # else:
        #     self.drivetrain.swerve_drive.set_state_std_devs(0.0, 0.0, 0.0)
        self._updateRobotPose()

    def setRobotOrientation(self) -> None:
        """Updates each Limelight with the robot's current orientation.

        Limelight's MegaTag2 localizer requires that we update it with our
        robot's latest yaw estimate periodically.
        """
        for ll in self._limelights:
            yaw = wpimath.inputModulus(self.drivetrain.swerve_drive.pigeon2.get_yaw().value, -180.0, 180.0)
            pitch = self.drivetrain.swerve_drive.pigeon2.get_pitch().value
            limelight.LimelightHelpers.set_robot_orientation(
                ll,
                yaw,
                0.0,
                pitch,
                0.0,
                0.0,
                0.0,
            )
    def _updateRobotPose(self) -> None:
        """Updates our robot pose estimate with the latest vision measurements."""
        for ll in self._limelights:
            pose_estimate: limelight.PoseEstimate = (
                limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
                    ll
                )
            )

            pose: wpimath.geometry.Pose2d = pose_estimate.pose
            drivetrain_pose = self.drivetrain.swerve_drive.get_state().pose

            vision_constants = self.robot_constants.drivetrain.vision

            # Filter out bad readings
            
            if not (pose_estimate.tag_count > 0):
                self.logger.warning("Rejected vision estimate: No tags seen")
                continue

            if (
                pose_estimate.avg_tag_dist
                > vision_constants.average_tag_distance_threshold
            ):
                self.logger.warning(
                    f"{ll}: Rejected too far away pose: {pose_estimate.avg_tag_dist}"
                )
                continue

            if (
                pose.X() < vision_constants.pose_x_min
                or pose.X() > vision_constants.pose_x_max
            ) or (
                pose.Y() < vision_constants.pose_y_min
                or pose.Y() > vision_constants.pose_y_max
            ):
                self.logger.warning(
                    f"{ll}: Rejected out-of-bounds pose: ({pose.X()}, {pose.Y()})"
                )
                continue

            # If this is the first good vision estimate since startup, hard
            # reset the translation portion of the drivetrain's estimate to it.
            if not self._pose_seeded:
                self.drivetrain.setPose(pose)
                self._pose_seeded = True
                self.logger.info(
                    f"Pose seeded from {ll}: ({pose.X()}, {pose.Y()})"
                )
                continue

            synced_timestamp = utils.fpga_to_current_time(
                pose_estimate.timestamp_seconds
            )
            self.drivetrain.swerve_drive.add_vision_measurement(
                pose_estimate.pose,
                synced_timestamp,
                (
                    0.5,
                    0.5,
                    math.inf
                ),
            )

    def set_std_devs(self, xy_std_dev, theta_std_dev) -> None:
        self.xy_std_devs = xy_std_dev
        self.theta_std_devs = theta_std_dev

    @feedback
    def get_robot_pose(self) -> wpimath.geometry.Pose2d:
        """
        Returns robot pose as a Pose2d object.
        """
        return self.drivetrain.swerve_drive.get_state().pose

    @feedback
    def get_drivetrain_yaw_degrees(self) -> float:
        """
        Returns drivetrain's yaw estimate.
        """
        return (
            self.drivetrain.swerve_drive.get_state().pose.rotation().degrees()
        )


    @feedback
    def get_limelight_upfr(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue(
            "limelight-upfr"
        )
    
    @feedback
    def get_limelight_upfl(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue(
            "limelight-upfl"
        )
    
    @feedback
    def get_limelight_fr(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue(
            "limelight-fr"
        )
    
    @feedback
    def get_limelight_fl(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue(
            "limelight-fl"
        )
    

    @feedback
    def get_limelight_upfr_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-upfr").avg_tag_dist
    
    @feedback
    def get_limelight_upfr_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-upfr").tag_count
    
    @feedback
    def get_limelight_upfl_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-upfl").avg_tag_dist
    
    @feedback
    def get_limelight_upfl_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-upfl").tag_count
    
    @feedback
    def get_limelight_fr_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-fr").avg_tag_dist
    
    @feedback
    def get_limelight_fr_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-fr").tag_count
    
    @feedback
    def get_limelight_fl_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-fl").avg_tag_dist
    
    @feedback
    def get_limelight_fl_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2("limelight-fl").tag_count


class VisionTuner:
    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain
    vision: Vision

    xy_std_devs = tunable(0.0)
    theta_std_devs = tunable(0.0)

    def setup(self) -> None:
        self.xy_std_devs = 0.0
        self.last_xy_std_devs = self.xy_std_devs

        self.theta_std_devs = 0.0
        self.last_theta_std_devs = self.theta_std_devs
        self._limelights: list[str] = (
            self.robot_constants.drivetrain.vision.limelights
        )
        self.nt = NetworkTableInstance.getDefault()


    def execute(self) -> None:
        if not self.valuesChanged():
            return
        if wpilib.DriverStation.isDisabled():
            self.throttleLimelights(True)
        else:
            self.throttleLimelights(False)

        self.applyValues()

        self.last_xy_std_devs = self.xy_std_devs
        self.last_theta_std_devs = self.theta_std_devs

    def valuesChanged(self) -> bool:
        return (
            self.last_xy_std_devs != self.xy_std_devs
            or self.last_theta_std_devs != self.theta_std_devs
        )

    def applyValues(self) -> None:
        self.vision.set_std_devs(self.xy_std_devs, self.theta_std_devs)

    def throttleLimelights(self, on: bool) -> None:
        if on:
            for ll in self._limelights:
                limelight.LimelightHelpers.set_LED_to_force_off(ll)
                self.nt.getTable(ll).getEntry("throttle_set").setInteger(150)
        else:
            for ll in self._limelights:
                limelight.LimelightHelpers.set_LED_to_pipeline_control(ll)
                self.nt.getTable(ll).getEntry("throttle_set").setInteger(0)
                


                


    
