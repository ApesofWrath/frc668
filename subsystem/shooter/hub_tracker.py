import magicbot
import phoenix6
from wpimath import geometry, units

import constants
from subsystem import drivetrain, shooter

INCHES_TO_METERS = 0.0254
TURRET_TO_ROBOT_X = 4.25 * INCHES_TO_METERS
TURRET_TO_ROBOT_Y = 0.0 * INCHES_TO_METERS
HUB_TO_FIELD_X = 182.11 * INCHES_TO_METERS
HUB_TO_FIELD_Y = 158.845 * INCHES_TO_METERS


class HubTracker:
    """Controls our shooter mechanisms to track the hub as the robot moves.

    The turret is always commanded to point at the center of the hub. The hood
    angle and flywheel speed are looked up based on our current position on the
    field.
    """

    robot_constants: constants.RobotConstants
    drivetrain: drivetrain.Drivetrain
    turret: shooter.Turret

    def setup(self) -> None:
        # Transform from robot frame to turret frame.
        self._robot_to_turret_transform: geometry.Transform2d = (
            geometry.Transform2d(
                geometry.Translation2d(TURRET_TO_ROBOT_X, TURRET_TO_ROBOT_Y),
                geometry.Rotation2d(),
            )
        )
        # Vector from field origin to center of the hub.
        self._hub_position: geometry.Translation2d = geometry.Translation2d(
            HUB_TO_FIELD_X, HUB_TO_FIELD_Y
        )
        # Pose of the turret relative to the field. This will be computed each
        # control loop based on the current robot pose estimate.
        self._turret_field_pose: geometry.Pose2d = geometry.Pose2d()

        # Raw yaw rate of the robot (and the turret).
        self._yaw_rate_signal: phoenix6.status_signal.StatusSignal[
            phoenix6.units.degrees_per_second
        ] = self.drivetrain.pigeon2.get_angular_velocity_z_world()

    def execute(self) -> None:
        self._yaw_rate_signal.refresh()

        # Pose of the robot relative to field origin.
        robot_pose: geometry.Pose2d = self.drivetrain.get_state().pose
        self._turret_field_pose: geometry.Pose2d = robot_pose.transformBy(
            self._robot_to_turret_transform
        )

        # Predict how much the robot will yaw in the next control loop interval
        # based on our current yaw rate.
        predictive_lead_angle = self._yaw_rate_signal.value * 0.02

        turret_target_angle = max(
            self.robot_constants.shooter.turret.min_angle,
            min(
                self.robot_constants.shooter.turret.max_angle,
                self.get_turret_target_angle_degrees() - predictive_lead_angle,
            ),
        )
        self.turret.setPosition(turret_target_angle)

        # TODO: Command the hood and the flywheel based on lookup table.

    @magicbot.feedback
    def get_turret_distance_from_hub_meters(self) -> units.meters:
        """Returns the absolute distance of the turret from the hub."""
        # Vector from center of turret to center of hub.
        turret_to_hub = (
            self._hub_position - self._turret_field_pose.translation()
        )
        return turret_to_hub.norm()

    @magicbot.feedback
    def get_turret_target_angle_degrees(self) -> units.degrees:
        """Returns the target angle for the turret to track.

        Tracking this angle ensures that the turret is always pointed at the
        hub.
        """
        # Vector from center of turret to center of hub.
        turret_to_hub = (
            self._hub_position - self._turret_field_pose.translation()
        )
        return (
            turret_to_hub.angle() - self._turret_field_pose.rotation()
        ).degrees()
