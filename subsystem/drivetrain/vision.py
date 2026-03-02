import math

import wpimath
from magicbot import feedback
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
        # Tracks whether the drivetrain's pose estimator has been seeded with a
        # good vision estimate since startup.
        self._pose_seeded = False

    def execute(self) -> None:
        self.setRobotOrientation()
        self._updateRobotPose()

    def setRobotOrientation(self) -> None:
        """Updates each Limelight with the robot's current orientation.

        Limelight's MegaTag2 localizer requires that we update it with our
        robot's latest yaw estimate periodically.
        """
        orientation: float = (
            self.drivetrain.get_state().pose.rotation().degrees()
        )

        for ll in self._limelights:
            limelight.LimelightHelpers.set_robot_orientation(
                ll,
                orientation,
                0.0,
                0.0,
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
            drivetrain_pose = self.drivetrain.get_state().pose

            vision_constants = self.robot_constants.drivetrain.vision

            # Filter out bad readings
            if not (pose_estimate.tag_count > 0):
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
                # self.drivetrain.reset_translation(pose.translation())
                self.drivetrain.reset_pose(pose)
                self._pose_seeded = True
                self.logger.info(
                    f"Pose seeded from {ll}: ({pose.X()}, {pose.Y()})"
                )
                continue

            if (
                drivetrain_pose.translation().distance(pose.translation())
                > vision_constants.max_diff_from_robot_pose
            ):
                self.logger.warning(
                    f"{ll}: Rejected large jump: {drivetrain_pose.translation().distance(pose.translation())}m"
                )
                continue

            # Nudge the drivetrain's pose estimator in the direction of the
            # vision estimate.

            # Dynamic std devs for translation.
            #
            # Accuracy of position decreases exponentially with distance:
            # According to Abbas et al. (2019) in Sensors, AprilTag precision
            # degrades by multiple manifolds as a function of distance and yaw
            # angle, with errors spiking from sub-cm levels to over 100cm [1.1].
            #
            # The research indicates that a non-linear probabilistic sensor
            # model using Gaussian Processes is required to account for the
            # rapid, non-linear increase in uncertainty at range [1.1]. Read the
            # full analysis at PMC6960891.
            #
            # Divide by tag count since more visible tags means higher
            # certainty.
            xy_std_devs = (
                pose_estimate.avg_tag_dist**2
            ) / pose_estimate.tag_count

            synced_timestamp = utils.fpga_to_current_time(
                pose_estimate.timestamp_seconds
            )
            self.drivetrain.add_vision_measurement(
                pose_estimate.pose,
                synced_timestamp,
                (xy_std_devs, xy_std_devs, math.inf),
            )

    @feedback
    def get_robot_pose(self) -> wpimath.geometry.Pose2d:
        """
        Returns robot pose as a Pose2d object.
        """
        return self.drivetrain.get_state().pose

    @feedback
    def get_robot_yaw_degrees(self) -> float:
        """
        Returns drivetrain's yaw estimate.
        """
        return self.drivetrain.get_state().pose.rotation().degrees()
