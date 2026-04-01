import math

import magicbot
import ntcore
import wpilib
import wpimath
from phoenix6 import utils

import constants
from common import datalog
from subsystem import drivetrain
from subsystem.drivetrain import limelight

from ntcore import NetworkTableInstance

import wpilib

RADIANS_TO_DEGREES = 180.0 / math.pi
DEGREES_TO_RADIANS = math.pi / 180.0


class Vision:
    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain
    data_logger: datalog.DataLogger

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

        # Publishers for accepted and rejected pose estimates.
        self._accepted_pose_publisher = nt.getStructArrayTopic(
            "/components/vision/accepted_pose_estimates",
            wpimath.geometry.Pose2d,
        ).publish()
        self._rejected_pose_publisher = nt.getStructArrayTopic(
            "/components/vision/rejected_pose_estimates",
            wpimath.geometry.Pose2d,
        ).publish()

        self._nt = ntcore.NetworkTableInstance.getDefault()
        self._last_throttle_state: bool = False



    def execute(self) -> None:
        is_disabled = wpilib.DriverStation.isDisabled()
        if is_disabled != self._last_throttle_state:
            self.throttleLimelights(is_disabled)
            self._last_throttle_state = is_disabled
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
            limelight.LimelightHelpers.set_robot_orientation(
                ll,
                self.drivetrain.estimatedYawDegrees(),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
    
    def avg_dist_stds(self, distance) -> float:
        return ((math.pow(distance, 2) / 23) - (distance/10) + 0.16)
    
    def rot_vel_stds(self, rot_vel) -> float:
        return (((math.pow(rot_vel, 2)) / (10 * math.pi)) + 0.05)
    
    def vel_stds(self, vel) -> float:
        return (math.pow(vel, 2) / 3)
    
    def tag_factor(n):
        return max(0.5, 1.0 / math.sqrt(max(1, n)))

    def _updateRobotPose(self) -> None:
        """Updates our robot pose estimate with the latest vision measurements."""
        rejected_poses: list[wpimath.geometry.Pose2d] = []
        rejected_limelights: list[str] = []
        rejected_reasons: list[str] = []

        accepted_poses: list[wpimath.geometry.Pose2d] = []
        accepted_limelights: list[str] = []

        drivetrain_pose = self.drivetrain.get_robot_pose()
        drivetrain_velocity = self.drivetrain.robotSpeeds()
        drivetrain_yaw = self.drivetrain.swerve_drive.pigeon2.get_angular_velocity_z_world().value * DEGREES_TO_RADIANS
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

            """
            @param avg_distance
            @param drivetrain_velocty / drivetrain_rotation_velocity
            @param number_tags_seen
            """
            
            

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


            # if (
            #     drivetrain_pose.translation().distance(pose.translation())
            #     > vision_constants.max_diff_from_robot_pose
            # ):
            #     self.logger.warning(
            #         f"{ll}: Rejected large jump: {drivetrain_pose.translation().distance(pose.translation())}m"
            #     )
            #     continue

            accepted_poses.append(pose)
            accepted_limelights.append(ll)

            synced_timestamp = utils.fpga_to_current_time(
                pose_estimate.timestamp_seconds
            )

            velocity_stds = self.vel_stds(drivetrain_velocity)
            average_distance_stds = self.avg_dist_stds(pose_estimate.avg_tag_dist)
            rotational_velocity_stds = self.rot_vel_stds(drivetrain_yaw)
            factor = self.tag_factor()

            self._xy_std_dev = average_distance_stds + 0.5 * velocity_stds + 0.2 * rotational_velocity_stds
            self._theta_std_dev = self.rotational_velocity_stds + 0.5 * self.average_distance_stds

            self._xy_std_dev *= factor
            self._theta_std_dev *= max(0.7, factor)

            self._xy_std_dev = min(max(self._xy_std_dev, 0.1), 3.0)
            self._theta_std_dev = min(max(self._theta_std_dev, 0.05), 2.0)

            self.drivetrain.swerve_drive.add_vision_measurement(
                pose_estimate.pose,
                synced_timestamp,
                (self._xy_std_dev, self._xy_std_dev, self._theta_std_dev),
            )

        self._accepted_pose_publisher.set(accepted_poses)
        self.data_logger.logStringArray(
            "/components/vision/accepted_limelights", accepted_limelights
        )
        self._rejected_pose_publisher.set(rejected_poses)
        self.data_logger.logStringArray(
            "/components/vision/rejected_limelights", rejected_limelights
        )
        self.data_logger.logStringArray(
            "/components/vision/rejected_reasons", rejected_reasons
        )


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

    @magicbot.feedback
    def get_limelight_fl(self) -> wpimath.geometry.Pose2d:
        return limelight.LimelightHelpers.get_botpose_2d_wpiblue("limelight-fl")




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

    def execute(self) -> None:
        self.vision.setStdDevs(self.xy_std_dev, self.theta_std_dev)

