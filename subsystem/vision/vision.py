import math

import wpimath

from subsystem import drivetrain, vision

import phoenix6

from phoenix6 import utils

from magicbot import feedback

RADIANS_TO_DEGREES = 180.0 / math.pi


class Vision:
    drivetrain: drivetrain.Drivetrain

    def setup(self) -> None:
        """
        Sets up varibles and has a list with limelight names,
        need to manually add more with more limelights.
        """
        self._limelights: list[str] = []
        self._limelights.append(vision.constants.LIMELIGHT_ONE)
        self._limelights.append(vision.constants.LIMELIGHT_TWO)

        self._pose_seeded = False
        self._imu_mode_is_four = False

    def execute(self) -> None:
        self._setRobotOrientation()
        self._updateRobotPose()

    def _setRobotOrientation(self) -> None:
        """Updates each Limelight with the robot's current orientation.

        Limelight's MegaTag2 localizer requires that we update it with our
        robot's latest yaw estimate periodically.
        """
        orientation: float = self.drivetrain.get_state().pose.rotation().degrees()

        for ll in self._limelights:
            vision.limelight.LimelightHelpers.set_robot_orientation(
                ll,
                orientation,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )

            # Makes limelights use internal IMU
            if not self._imu_mode_is_four:
                for ll in self._limelights:
                    vision.limelight.LimelightHelpers.set_imu_mode(ll)
                self._imu_mode_is_four = True
                self.logger.info("Set Limelight's IMUs to mode: 4")

    def _updateRobotPose(self) -> None:
        """Updates our robot pose estimate with the latest vision measurements."""
        for ll in self._limelights:
            pose_estimate: vision.limelight.PoseEstimate = (
                vision.limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(ll)
            )

            pose = pose_estimate.pose
            drivetrain_pose = self.drivetrain.get_state().pose

            # Filters out bad readings
            if not (pose_estimate.tag_count > 0):
                continue

            if pose_estimate.avg_tag_dist > vision.constants.AVG_TAG_DISTANCE_THRESHOLD:
                self.logger.warning(f"{ll}: Rejected too far away pose: {pose_estimate.avg_tag_dist}")
                continue

            if (pose.X() < vision.constants.POSE_X_MIN or pose.X() > vision.constants.POSE_X_MAX) or (pose.Y() < - vision.constants.POSE_Y_MIN or pose.Y() > vision.constants.POSE_Y_MAX):
                self.logger.warning(f"{ll}: Rejected out-of-bounds pose: ({pose.X()}, {pose.Y()})")
                continue

            if not self._pose_seeded:
                self.drivetrain.reset_pose(pose)
                self._pose_seeded = True
                self.logger.info(f"Pose seeded from {ll}: ({pose.X()}, {pose.Y()})")
                continue

            if drivetrain_pose.translation().distance(pose.translation()) > vision.constants.MAX_DIFF_FROM_ROBOT_POSE:
                self.logger.warning(f"{ll}: Rejected large jump: {drivetrain_pose.translation().distance(pose.translation())}m")
                continue

            # Dynamic stds for position, could add for rotation
            xy_std_devs = (pose_estimate.avg_tag_dist ** 2) / pose_estimate.tag_count
            

            # Adds vision measurement
            self._last_known_pose = pose
            synced_timestamp = utils.fpga_to_current_time(pose_estimate.timestamp_seconds)
            self.drivetrain.add_vision_measurement(
                pose_estimate.pose, synced_timestamp, (xy_std_devs, xy_std_devs, math.inf)
            )


    @feedback
    def get_robot_pose(self) -> wpimath.geometry.Pose2d:
        """
        Returns robot pose as a Pose2d object which can be vizualized on Advantage Scope
        """
        pose: wpimath.geometry.Pose2d = self.drivetrain.get_state().pose
        if pose is not None:
            return pose
        return wpimath.geometry.Pose2d(self._last_known_pose)

    @feedback
    def get_gyro(self) -> float:
        """
        Returns drivetrain's yaw, not the actual gyro readings
        """
        return self.drivetrain.get_state().pose.rotation().degrees()