import math

import wpimath

from subsystem import drivetrain, vision

RADIANS_TO_DEGREES = 180.0 / math.pi


class Vision:
    drivetrain: drivetrain.Drivetrain

    def setup(self) -> None:
        self._limelights: list[str] = []
        self._limelights.append(vision.constants.LIMELIGHT_ONE)
        self._limelights.append(vision.constants.LIMELIGHT_TWO)

    def execute(self) -> None:
        self._setRobotOrientation()
        self._updateRobotPose()

    def _setRobotOrientation(self) -> None:
        """Updates each Limelight with the robot's current orientation.

        Limelight's MegaTag2 localizer requires that we update it with our
        robot's latest yaw estimate periodically.
        """
        orientation: wpimath.geometry.Rotation3d = (
            self.drivetrain.get_rotation3d()
        )
        for ll in self._limelights:
            vision.limelight.LimelightHelpers.set_robot_orientation(
                ll,
                orientation.Z() * RADIANS_TO_DEGREES,
                0.0,
                orientation.Y() * RADIANS_TO_DEGREES,
                0.0,
                orientation.X() * RADIANS_TO_DEGREES,
                0.0,
            )

    def _updateRobotPose(self) -> None:
        """Updates our robot pose estimate with the latest vision measurements."""
        for ll in self._limelights:
            pose_estimate: vision.limelight.PoseEstimate = (
                vision.limelight.LimelightHelpers.get_botpose_estimate_wpiblue_megatag2(
                    ll
                )
            )
            # TODO: Filter the bad estimates out.
            if pose_estimate.tag_count > 0:
                self.drivetrain.add_vision_measurement(
                    pose_estimate.pose, pose_estimate.timestamp_seconds
                )

    @magicbot.feedback
    def get_robot_pose(self) -> list[float]:
        pose: wpimath.geometry.Pose2d = self.drivetrain.sample_pose_at(
            phoenix6.utils.get_current_time_seconds()
        )
        return [pose.X(), pose.Y(), pose.rotation().degrees()]
