import math

import magicbot
import ntcore
import wpilib
import wpimath
from phoenix6 import utils

import constants
from subsystem import drivetrain
from subsystem.drivetrain import limelight

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

        self._xy_std_dev = self.robot_constants.drivetrain.vision.xy_std_dev
        self._theta_std_dev = (
            self.robot_constants.drivetrain.vision.theta_std_dev
        )

        nt = ntcore.NetworkTableInstance.getDefault()

        for ll in self._limelights:
            # Ensure that the limelights are not throttled.
            limelight.LimelightHelpers.set_LED_to_pipeline_control(ll)
            nt.getTable(ll).getEntry("throttle_set").setInteger(0)
            # Set how much the Limelight trusts the external IMU. This is only
            # relevant when using IMU mode 4.
            limelight.LimelightHelpers.set_limelight_NTDouble(
                ll, "imuassistalpha_set", 0.001
            )

        # Publishers for information about rejected pose estimates.
        self._rejected_pose_publisher = nt.getStructArrayTopic(
            "/components/vision/rejected_pose_estimates",
            wpimath.geometry.Pose2d,
        ).publish()
        self._rejected_limelights_publisher = nt.getStringArrayTopic(
            "/components/vision/rejected_limelights"
        ).publish()
        self._rejected_reasons_publisher = nt.getStringArrayTopic(
            "/components/vision/rejected_reasons"
        ).publish()

        # Publishers for information about accepted pose estimates.
        self._accepted_pose_publisher = nt.getStructArrayTopic(
            "/components/vision/accepted_pose_estimates",
            wpimath.geometry.Pose2d,
        ).publish()
        self._accepted_limelights_publisher = nt.getStringArrayTopic(
            "/components/vision/accepted_limelights"
        ).publish()

    def execute(self) -> None:
        self.setRobotOrientation()
        self._updateRobotPose()

    def setImuMode(self, value: int) -> None:
        if not isinstance(value, int) or value < 0 or value > 4:
            self.logger.warning(
                f"IMU mode must be an integer in the range [0, 4], got: {value}."
            )
            return
        for ll in self._limelights:
            limelight.LimelightHelpers.set_imu_mode(ll, value)

    def setRobotOrientation(self) -> None:
        """Updates each Limelight with the robot's current orientation.

        Limelight's MegaTag2 localizer requires that we update it with our
        robot's latest yaw estimate periodically.
        """
        for ll in self._limelights:
            orientation: float = (
            orientation = self.drivetrain.swerve_drive.get_state().pose.rotation().degrees()
        )
            pitch_roll = self.drivetrain.swerve_drive.get_rotation3d()
            limelight.LimelightHelpers.set_robot_orientation(
                ll,
                self.drivetrain.get_estimated_yaw_degrees(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
    def _updateRobotPose(self) -> None:
        """Updates our robot pose estimate with the latest vision measurements."""
        rejected_poses: list[wpimath.geometry.Pose2d] = []
        rejected_limelights: list[str] = []
        rejected_reasons: list[str] = []

        accepted_poses: list[wpimath.geometry.Pose2d] = []
        accepted_limelights: list[str] = []

        drivetrain_pose = self.drivetrain.get_robot_pose()
        vision_constants = self.robot_constants.drivetrain.vision

        for ll in self._limelights:
            pose_estimate: limelight.PoseEstimate = (
                limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
                    ll
                )
            )
            pose: wpimath.geometry.Pose2d = pose_estimate.pose

            # Filter out bad readings

            if not (pose_estimate.tag_count > 0):
                rejected_poses.append(pose)
                rejected_limelights.append(ll)
                rejected_reasons.append("No tags seen")
                continue

            if (
                pose_estimate.avg_tag_dist
                > vision_constants.average_tag_distance_threshold
            ):
                rejected_poses.append(pose)
                rejected_limelights.append(ll)
                rejected_reasons.append(
                    f"Too far away: {pose_estimate.avg_tag_dist:.2f}m"
                )
                continue

            if (
                pose.X() < vision_constants.pose_x_min
                or pose.X() > vision_constants.pose_x_max
            ) or (
                pose.Y() < vision_constants.pose_y_min
                or pose.Y() > vision_constants.pose_y_max
            ):
                rejected_poses.append(pose)
                rejected_limelights.append(ll)
                rejected_reasons.append(
                    f"Out of bounds: ({pose.X():.2f}, {pose.Y():.2f})"
                )
                continue

            accepted_poses.append(pose)
            accepted_limelights.append(ll)

            synced_timestamp = utils.fpga_to_current_time(
                pose_estimate.timestamp_seconds
            )
            self.drivetrain.swerve_drive.add_vision_measurement(
                pose_estimate.pose,
                synced_timestamp,
                (self._xy_std_dev, self._xy_std_dev, self._theta_std_dev),
            )

        self._accepted_pose_publisher.set(accepted_poses)
        self._accepted_limelights_publisher.set(accepted_limelights)
        self._rejected_pose_publisher.set(rejected_poses)
        self._rejected_limelights_publisher.set(rejected_limelights)
        self._rejected_reasons_publisher.set(rejected_reasons)

    def setStdDevs(self, xy_std_dev, theta_std_dev) -> None:
        self._xy_std_dev = xy_std_dev
        self._theta_std_dev = theta_std_dev

    @magicbot.feedback
    def get_limelight_fl_pose(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue("limelight-fl")

    @magicbot.feedback
    def get_limelight_fl_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-fl"
        ).avg_tag_dist

    @magicbot.feedback
    def get_limelight_fl_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-fl"
        ).tag_count

    @magicbot.feedback
    def get_limelight_fr_pose(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue("limelight-fr")

    @magicbot.feedback
    def get_limelight_fr_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-fr"
        ).avg_tag_dist

    @magicbot.feedback
    def get_limelight_fr_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-fr"
        ).tag_count

    @magicbot.feedback
    def get_limelight_upfl_pose(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue(
            "limelight-upfl"
        )

    @magicbot.feedback
    def get_limelight_upfl_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-upfl"
        ).avg_tag_dist

    @magicbot.feedback
    def get_limelight_upfl_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-upfl"
        ).tag_count

    @magicbot.feedback
    def get_limelight_upfr_pose(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue(
            "limelight-upfr"
        )

    @magicbot.feedback
    def get_limelight_upfr_avg_dist(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-upfr"
        ).avg_tag_dist

    @magicbot.feedback
    def get_limelight_upfr_tag_count(self):
        return limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
            "limelight-upfr"
        ).tag_count


class VisionTuner:
    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain
    vision: Vision

    xy_std_dev = magicbot.tunable(0.0)
    theta_std_dev = magicbot.tunable(0.0)

    def setup(self) -> None:
        self.xy_std_dev = self.robot_constants.drivetrain.vision.xy_std_dev
        self.theta_std_dev = (
            self.robot_constants.drivetrain.vision.theta_std_dev
        )

        self._limelights: list[str] = (
            self.robot_constants.drivetrain.vision.limelights
        )
        self._nt = ntcore.NetworkTableInstance.getDefault()

    def execute(self) -> None:
        self.vision.setStdDevs(self.xy_std_dev, self.theta_std_dev)

        if wpilib.DriverStation.isDisabled():
            self.throttleLimelights(True)
        else:
            self.throttleLimelights(False)

    def throttleLimelights(self, value: bool) -> None:
        """Throttle the limelights so they don't overheat."""
        if value:
            for ll in self._limelights:
                limelight.LimelightHelpers.set_LED_to_force_off(ll)
                self._nt.getTable(ll).getEntry("throttle_set").setInteger(150)
        else:
            for ll in self._limelights:
                limelight.LimelightHelpers.set_LED_to_pipeline_control(ll)
                self._nt.getTable(ll).getEntry("throttle_set").setInteger(0)
