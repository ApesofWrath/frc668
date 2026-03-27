import bisect
import math
from typing import Tuple

import magicbot
import phoenix6
import wpilib
from wpimath import geometry, units

import constants
from common import alliance, datalog
from subsystem import drivetrain, shooter

INCHES_TO_METERS = 0.0254
TURRET_TO_ROBOT_X = 4.25 * INCHES_TO_METERS
TURRET_TO_ROBOT_Y = 0.0 * INCHES_TO_METERS
RED_HUB_TO_FIELD_X = 469.11 * INCHES_TO_METERS
RED_HUB_TO_FIELD_Y = 158.845 * INCHES_TO_METERS
BLUE_HUB_TO_FIELD_X = 182.11 * INCHES_TO_METERS
BLUE_HUB_TO_FIELD_Y = 158.845 * INCHES_TO_METERS


class ShotTable:
    """A lookup table for hood angles and flywheel speeds based on robot pose."""

    # Each entry contains the following:
    #  * turret-to-hub distance in meters
    #  * hood angle in degrees
    #  * flywheel speed in rotations per second
    _TABLE: Tuple[Tuple[float, float, float], ...] = (
        (2.25, 2.0, 27.5),
        (2.75, 3.5, 29.0),
        (3.25, 5.0, 30.5),
        (3.75, 6.5, 32.5),
        (4.25, 7.0, 34.0),
        (4.75, 7.5, 36.0),
    )
    # A tuple of just the distances from _TABLE.
    _DISTANCES: Tuple[float, ...] = tuple(row[0] for row in _TABLE)

    @classmethod
    def get(cls, distance_meters: float) -> Tuple[float, float]:
        """Get (hood angle, flywheel speed) for a given turret-to-hub distance.

        This interpolates between values for known distances.

        Args:
            distance_meters:
                The Euclidian distance in the XY plane in meters from the center
                of the turret to the center of the hub.

        Returns:
            A tuple containing the target hood angle in degrees and flywheel
            speed in rotations per second.
        """
        if distance_meters <= cls._TABLE[0][0]:
            return cls._TABLE[0][1], cls._TABLE[0][2]
        if distance_meters >= cls._TABLE[-1][0]:
            return cls._TABLE[-1][1], cls._TABLE[-1][2]

        idx = bisect.bisect_left(cls._DISTANCES, distance_meters) - 1

        d1, h1, f1 = cls._TABLE[idx]
        d2, h2, f2 = cls._TABLE[idx + 1]

        fraction = (distance_meters - d1) / (d2 - d1)
        hood_angle = h1 + fraction * (h2 - h1)
        flywheel_speed = f1 + fraction * (f2 - f1)

        return hood_angle, flywheel_speed


class HubTracker:
    """Controls our shooter mechanisms to track the hub as the robot moves.

    The turret is always commanded to point at the center of the hub. The hood
    angle and flywheel speed are looked up based on our current position on the
    field.
    """

    robot_constants: constants.RobotConstants
    alliance_fetcher: alliance.AllianceFetcher
    drivetrain: drivetrain.Drivetrain
    flywheel: shooter.Flywheel
    hood: shooter.Hood
    turret: shooter.Turret
    data_logger: datalog.DataLogger

    def setup(self) -> None:
        # Transform from robot frame to turret frame.
        self._robot_to_turret_transform: geometry.Transform2d = (
            geometry.Transform2d(
                geometry.Translation2d(TURRET_TO_ROBOT_X, TURRET_TO_ROBOT_Y),
                geometry.Rotation2d(),
            )
        )
        # Vector from field origin to center of the hub.
        self._hub_position: geometry.Translation2d = (
            geometry.Translation2d(RED_HUB_TO_FIELD_X, RED_HUB_TO_FIELD_Y)
            if self.alliance_fetcher.isRedAlliance()
            else geometry.Translation2d(
                BLUE_HUB_TO_FIELD_X, BLUE_HUB_TO_FIELD_Y
            )
        )
        # Pose of the turret relative to the field. This will be computed each
        # control loop based on the current robot pose estimate.
        self._turret_field_pose: geometry.Pose2d = geometry.Pose2d()

        # Current targets.
        # TODO: Find better defaults.
        self._target_turret_angle_degrees: float = 0.0
        self._target_hood_angle_degrees: float = 0.0
        self._target_flywheel_speed_rps: float = 0.0

        self.turret_mvt_feed_forward_multiplier: float = (
            self.robot_constants.shooter.turret.feed_forward_mvt_multiplier
        )
        self.turret_mvt_feed_forward: float = 0.0

        self.current_turret_to_hub: geometry.Translation2d = (
            geometry.Translation2d(0, 0)
        )
        self.future_turret_to_hub: geometry.Translation2d = (
            geometry.Translation2d(0, 0)
        )

        # Raw yaw rate of the robot (and the turret).
        self._yaw_rate_signal: phoenix6.status_signal.StatusSignal[
            phoenix6.units.degrees_per_second
        ] = self.drivetrain.swerve_drive.pigeon2.get_angular_velocity_z_world()

        # Whether to re-calculate the turret and hood positions each loop.
        self._track_position = True
        # Whether to re-calculate the flywheel speed each loop.
        self._track_speed = False
        # Whether to command mechanisms to the current targets.
        self._enabled = True

    def execute(self) -> None:
        self._yaw_rate_signal.refresh()

        # Pose of the robot relative to field origin.
        robot_pose: geometry.Pose2d = self.drivetrain.get_robot_pose()
        self._turret_field_pose: geometry.Pose2d = robot_pose.transformBy(
            self._robot_to_turret_transform
        )

        self.turret_moving_target_angle = (
            self._computeMovingTargetTurretAngleDegrees()
        )

        # Set the flywheel and hood targets based on future turret-to-hub
        # distance.
        target_hood_angle_degrees, target_flywheel_speed_rps = ShotTable.get(
            self.futureTurretDistanceFromHubMeters()
        )

        self.turret_mvt_feed_forward = (
            self.turret_moving_target_angle
            - self._computeStationaryTargetTurretAngleDegrees()
        ) * self.turret_mvt_feed_forward_multiplier

        if self._track_position:
            self._target_turret_angle_degrees = self.turret_moving_target_angle
            self._target_hood_angle_degrees = target_hood_angle_degrees

        if self._track_speed:
            self._target_flywheel_speed_rps = target_flywheel_speed_rps

        if self._enabled:
            self.turret.setFeedForwardControl(self.turret_mvt_feed_forward)
            self.turret.setPosition(self._target_turret_angle_degrees)
            self.hood.setPosition(self._target_hood_angle_degrees)
            self.flywheel.setTargetRps(self._target_flywheel_speed_rps)

        self._logData()

    def trackPosition(self, value: bool) -> None:
        """If True, the turret and hood angles will be re-calculated each loop."""
        self._track_position = value

    def trackSpeed(self, value: bool) -> None:
        """If True, the flywheel speed will be re-calculated each loop."""
        self._track_speed = value

    def setTargetHoodAngleDegrees(self, value: float) -> None:
        """Set the target angle for the hood, in degrees."""
        self._target_hood_angle_degrees = value

    def setTargetTurretAngleDegrees(self, value: float) -> None:
        """Set the target angle for the turret, in degrees."""
        self._target_turret_angle_degrees = value

    def setTargetFlywheelSpeedRps(self, value: float) -> None:
        """Set the target speed for the flywheel, in rotations per second."""
        self._target_flywheel_speed_rps = value

    def setEnabled(self, value: bool) -> None:
        """If True, the mechanisms will be commanded to the current targets."""
        self._enabled = value

    def setTurretFeedForwardMultiplier(self, multiplier) -> None:
        self.turret_mvt_feed_forward_multiplier = multiplier

    def _computeMovingTargetTurretAngleDegrees(
        self,
    ) -> phoenix6.units.degree:
        """Computes turret angle to hit the target while moving.

        Takes into account both linear and angular velocities of the robot, and
        compensates for them.
        """
        # Vector from field origin to center of the hub. We set this again here
        # since we may not have known our alliance at startup.
        self._hub_position = (
            geometry.Translation2d(RED_HUB_TO_FIELD_X, RED_HUB_TO_FIELD_Y)
            if self.alliance_fetcher.isRedAlliance()
            else geometry.Translation2d(
                BLUE_HUB_TO_FIELD_X, BLUE_HUB_TO_FIELD_Y
            )
        )

        # Vector from center of turret to center of hub.
        self.future_turret_to_hub = self._hub_position - (
            self._turret_field_pose.translation() + self._getMovementVector()
        )
        # Heading of the turret_field_pose is same as the robot's heading.
        target_angle_degrees = (
            self.future_turret_to_hub.angle()
            - self._turret_field_pose.rotation()
        ).degrees()

        # Predict how much the robot will yaw in the next control loop interval
        # based on our current yaw rate.
        predictive_lead_angle = self._yaw_rate_signal.value * 0.02

        return max(
            self.robot_constants.shooter.turret.min_angle,
            min(
                self.robot_constants.shooter.turret.max_angle,
                target_angle_degrees - predictive_lead_angle,
            ),
        )

    def _computeStationaryTargetTurretAngleDegrees(
        self,
    ) -> phoenix6.units.degree:
        """Computes turret angle to hit the target while stationary.

        Compensates for robot's angular vecloity with a lookahead, but assumes
        robot's linear velocity is zero.
        """
        # Vector from field origin to center of the hub. We set this again here
        # since we may not have known our alliance at startup.
        self._hub_position = (
            geometry.Translation2d(RED_HUB_TO_FIELD_X, RED_HUB_TO_FIELD_Y)
            if self.alliance_fetcher.isRedAlliance()
            else geometry.Translation2d(
                BLUE_HUB_TO_FIELD_X, BLUE_HUB_TO_FIELD_Y
            )
        )

        # Vector from center of turret to center of hub.
        self.current_turret_to_hub = (
            self._hub_position - self._turret_field_pose.translation()
        )
        # Heading of the turret_field_pose is same as the robot's heading.
        target_angle_degrees = (
            self.current_turret_to_hub.angle()
            - self._turret_field_pose.rotation()
        ).degrees()

        # Predict how much the robot will yaw in the next control loop interval
        # based on our current yaw rate.
        predictive_lead_angle = self._yaw_rate_signal.value * 0.02

        return max(
            self.robot_constants.shooter.turret.min_angle,
            min(
                self.robot_constants.shooter.turret.max_angle,
                target_angle_degrees - predictive_lead_angle,
            ),
        )

    def _getMovementVector(self) -> geometry.Translation2d:
        """Computes distance vector of robot's movement.

        Uses a fixed time period (average time-of-flight of fuel) and the the
        robot's current velocity to determine the vector that represents the
        magnitude and direction of the distance traveled by the robot.

        This is meant to represent the distance the fuel will travel along the
        direction of the robot's velocity over its time-of-flight.
        """
        if self.alliance_fetcher.isRedAlliance():
            sign_multiplier = -1
        else:
            sign_multiplier = 1
        robot_vx = (
            sign_multiplier * self.drivetrain.swerve_drive.get_state().speeds.vx
        )
        robot_vy = (
            sign_multiplier * self.drivetrain.swerve_drive.get_state().speeds.vy
        )
        robot_omega = self.drivetrain.swerve_drive.get_state().speeds.omega
        robot_angle = (
            self.drivetrain.swerve_drive.get_state().pose.rotation().radians()
        )

        # The turret inherits some linear velocity from the robot's rate of
        # rotation, due to being offset from the robot's center of rotation.
        turret_vx = robot_vx + robot_omega * (
            TURRET_TO_ROBOT_Y * math.cos(robot_angle)
            - TURRET_TO_ROBOT_X * math.sin(robot_angle)
        )
        turret_vy = robot_vy + robot_omega * (
            TURRET_TO_ROBOT_X * math.cos(robot_angle)
            - TURRET_TO_ROBOT_Y * math.sin(robot_angle)
        )

        return (
            geometry.Translation2d(turret_vx, turret_vy)
        ) * self.robot_constants.shooter.turret.time_of_flight

    def currentTurretDistanceFromHubMeters(self) -> phoenix6.units.meter:
        """Returns the current absolute distance of the turret from the hub."""
        # Vector from center of turret to center of hub.
        current_turret_to_hub = (
            self._hub_position - self._turret_field_pose.translation()
        )
        return current_turret_to_hub.norm()

    def futureTurretDistanceFromHubMeters(self) -> phoenix6.units.meter:
        return self.future_turret_to_hub.norm()

    def futureTurretAngleToHub(self) -> phoenix6.units.degree:
        return self.future_turret_to_hub.angle().degrees()

    def _logData(self) -> None:
        self.data_logger.logBoolean(
            "/components/hub_tracker/enabled", self._enabled, on_change=True
        )
        self.data_logger.logBoolean(
            "/components/hub_tracker/track_position",
            self._track_position,
            on_change=True,
        )
        self.data_logger.logBoolean(
            "/components/hub_tracker/track_speed",
            self._track_speed,
            on_change=True,
        )
        self.data_logger.logDouble(
            "/components/hub_tracker/current_turret_distance_from_hub_meters",
            self.currentTurretDistanceFromHubMeters(),
        )
        self.data_logger.logDouble(
            "/components/hub_tracker/future_turret_distance_from_hub_meters",
            self.futureTurretDistanceFromHubMeters(),
        )
        self.data_logger.logDouble(
            "/components/hub_tracker/future_turret_to_hub_angle",
            self.futureTurretAngleToHub(),
        )
        self.data_logger.logDouble(
            "/components/hub_tracker/target_turret_angle_degrees",
            self._target_turret_angle_degrees,
        )
        self.data_logger.logDouble(
            "/components/hub_tracker/target_hood_angle_degrees",
            self._target_hood_angle_degrees,
        )
        self.data_logger.logDouble(
            "/components/hub_tracker/target_flywheel_speed_rps",
            self._target_flywheel_speed_rps,
        )
