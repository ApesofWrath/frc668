import math

import magicbot
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
        # Tracks whether the drivetrain's pose estimator has been seeded with a
        # good vision estimate since startup.
        self._pose_seeded = False

        self.xy_std_devs = 0.0
        self.theta_std_devs = 0.0

        self._pose_estimates: dict[str, wpimath.geometry.Pose2d] = dict(
            [(ll, wpimath.geometry.Pose2d()) for ll in self._limelights]
        )

    def execute(self) -> None:
        self.setRobotOrientation()
        # self._updateRobotPose()

    def setRobotOrientation(self) -> None:
        """Updates each Limelight with the robot's current orientation.

        Limelight's MegaTag2 localizer requires that we update it with our
        robot's latest yaw estimate periodically.
        """
        orientation: float = (
            self.drivetrain.swerve_drive.get_state().pose.rotation().degrees()
        )

        pitch_roll = self.drivetrain.swerve_drive.get_rotation3d()

        for ll in self._limelights:
            limelight.LimelightHelpers.set_robot_orientation(
                ll,
                orientation,
                0.0,
                pitch_roll.Y() * RADIANS_TO_DEGREES,
                0.0,
                pitch_roll.X() * RADIANS_TO_DEGREES,
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
            self._pose_estimates[ll] = pose
            drivetrain_pose = self.drivetrain.swerve_drive.get_state().pose

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
                self.drivetrain.setPose(pose)
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
            # xy_std_devs = (
            #     pose_estimate.avg_tag_dist**2
            # ) / pose_estimate.tag_count
            # theta_std_dev = math.inf

            synced_timestamp = utils.fpga_to_current_time(
                pose_estimate.timestamp_seconds
            )
            self.drivetrain.swerve_drive.add_vision_measurement(
                pose_estimate.pose,
                synced_timestamp,
                (
                    vision_constants.xy_std_dev,
                    vision_constants.xy_std_dev,
                    vision_constants.theta_std_dev,
                ),
            )

    def set_std_devs(self, xy_std_dev, theta_std_dev) -> None:
        self.xy_std_devs = xy_std_dev
        self.theta_std_devs = theta_std_dev

    @magicbot.feedback
    def get_limelight_fl_pose(self) -> wpimath.geometry.Pose2d:
        return self._pose_estimates["limelight-fl"]

    @magicbot.feedback
    def get_limelight_fr_pose(self) -> wpimath.geometry.Pose2d:
        return self._pose_estimates["limelight-fr"]

    @magicbot.feedback
    def get_limelight_upfl_pose(self) -> wpimath.geometry.Pose2d:
        return self._pose_estimates["limelight-upfl"]

    @magicbot.feedback
    def get_limelight_upfr_pose(self) -> wpimath.geometry.Pose2d:
        return self._pose_estimates["limelight-upfr"]

    @magicbot.feedback
    def get_xy_std_deviation(self) -> float:
        return self.xy_std_devs

    @magicbot.feedback
    def get_theta_std_deviation(self) -> float:
        return self.theta_std_devs


class VisionTuner:
    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain
    vision: Vision

    xy_std_devs = magicbot.tunable(0.0)
    theta_std_devs = magicbot.tunable(0.0)

    def setup(self) -> None:
        self.xy_std_devs = 0.0
        self.last_xy_std_devs = self.xy_std_devs

        self.theta_std_devs = 0.0
        self.last_theta_std_devs = self.theta_std_devs

    def execute(self) -> None:
        if not self.valuesChanged():
            return

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
