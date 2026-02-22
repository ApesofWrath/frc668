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
        self.imu_four = False

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
            if not self.imu_four:
                vision.limelight.LimelightHelpers.set_imu_mode(self._limelights[0], 4)
                vision.limelight.LimelightHelpers.set_imu_mode(self._limelights[1], 4)
                self.imu_four = True
                self.logger.info("Set Limelight IMU's to mode: 4")

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

            if pose_estimate.avg_tag_dist > 4.5:
                self.logger.warning(f"{ll}: Rejected too far away pose: {pose_estimate.avg_tag_dist}")
                continue

            if (pose.X() < -0.2 or pose.X() > 16.55) or (pose.Y() < -0.2 or pose.Y() > 8.07):
                self.logger.warning(f"{ll}: Rejected out-of-bounds pose: ({pose.X()}, {pose.Y()})")
                continue

            if not self._pose_seeded:
                self.drivetrain.reset_pose(pose)
                self._pose_seeded = True
                self.logger.info(f"Pose seeded from {ll}: ({pose.X()}, {pose.Y()})")
                continue

            if drivetrain_pose.translation().distance(pose.translation()) > 0.5:
                self.logger.warning(f"{ll}: Rejected large jump: {drivetrain_pose.translation().distance(pose.translation())}m")
                continue

            # Dynamic stds for position, could add for rotation
            xy_std_devs = (pose_estimate.avg_tag_dist ** 2) / pose_estimate.tag_count

            # Attempt at semi-dynamic rotation stds which failed
            # if pose_estimate.avg_tag_dist < 2.0:
            #     theta_std_devs = 0.9
            #     self.logger.info("Set std to 0.9")
            # else:
            #     theta_std_devs = math.inf
            #     self.logger.info("Set std to inf")

            # Adds vision measurement
            synced_timestamp = utils.fpga_to_current_time(pose_estimate.timestamp_seconds)
            self.drivetrain.add_vision_measurement(
                pose_estimate.pose, synced_timestamp, (xy_std_devs, xy_std_devs, math.inf)
            )

            # Commented out because it filled log file with messages
            # self.logger.info(f"Std dev values: {xy_std_devs}")
            # self.logger.info("Added vision measurement")

    @feedback
    def get_robot_pose(self) -> wpimath.geometry.Pose2d:
        """
        Returns robot pose as a Pose2d object which can be vizualized on Advantage Scope
        """
        pose: wpimath.geometry.Pose2d = self.drivetrain.get_state().pose
        if pose is not None:
            return pose
        return wpimath.geometry.Pose2d(
            wpimath.geometry.Translation2d(-10.0, -10.0),
            wpimath.geometry.Rotation2d(),
        )

    @feedback
    def get_gyro(self) -> float:
        """
        Returns drivetrain's yaw, not the actual gyro readings
        """
        return self.drivetrain.get_state().pose.rotation().degrees()