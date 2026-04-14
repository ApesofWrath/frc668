import math

import magicbot
from wpimath import kinematics

import constants
from common import datalog
from subsystem import shooter, drivetrain


class Shooter(magicbot.StateMachine):
    """State machine for the shooter.

    It only feeds fuel into the shooter once we have a good lock on the target
    and when the robot isn't moving.

    It also idles the flywheel when the driver doesn't want to shoot.

                     --------
                    | IDLING | <--
                     --------      \
                        ^ |         \
                        | |          \
                        | v           \
                    -----------  -->  ----------
                   | TARGETING |     | SHOOTING |
                    -----------  <--  ----------
    """

    turret: shooter.Turret
    hood: shooter.Hood
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    drivetrain: drivetrain.Drivetrain
    target_tracker: shooter.TargetTracker
    robot_constants: constants.RobotConstants
    data_logger: datalog.DataLogger

    def setup(self) -> None:
        self._driver_wants_feed = False
        self._auto = True

    @magicbot.state(first=True)
    def idling(self):
        """Waiting for the driver to command fuel feed."""
        if self._driver_wants_feed:
            self.next_state("targeting")

        # Idle the flywheel to save power, but have the turret and hood track
        # position.
        self.target_tracker.track_position(self._auto)
        self.target_tracker.track_speed(False)
        self.target_tracker.set_target_flywheel_speed_rps(
            self.robot_constants.shooter.flywheel.default_speed_rps
        )

        # Don't feed fuel.
        self.hopper.set_enabled(False)
        self.indexer.set_enabled(False)

    @magicbot.state
    def targeting(self):
        """Attempting to get to target state.

        This means the hood and turret must be close to their target angles, the
        flywheel must be close to its target speed, and the robot is stationary.
        """
        # First, check if the driver wants to shoot. If not, idle to save power.
        if not self._driver_wants_feed:
            self.next_state("idling")

        # Then, check if we are ready to shoot.
        if self._shooter_is_ready():
            self.next_state_now("shooting")

        # The driver wants to shoot but the shooter isn't ready or the robot is
        # moving. We want to continue tracking the target, but don't feed fuel.
        self.target_tracker.track_position(self._auto)
        self.target_tracker.track_speed(self._auto)

        self.hopper.set_enabled(False)
        self.indexer.set_enabled(False)

    @magicbot.state
    def shooting(self):
        """Actively shooting.

        Continue tracking the target, and feed fuel.
        """
        if not self._driver_wants_feed:
            self.next_state("idling")

        if not self._shooter_is_ready():
            self.next_state("targeting")

        # Fully track the target.
        self.target_tracker.track_position(self._auto)
        self.target_tracker.track_speed(self._auto)

        # Feed fuel.
        self.hopper.set_enabled(True)
        self.indexer.set_enabled(True)

    def set_driver_wants_feed(self, value: bool) -> None:
        self._driver_wants_feed = value

    def set_auto(self, value: bool) -> None:
        self._auto = value

    def _shooter_is_ready(self) -> bool:
        """Indicates if shooter components are close enough to their targets."""
        # These values were roughly tuned during drive testing.
        return self._get_shooter_is_within(
            turret_tolerance_degrees=6.0,
            hood_tolerance_degrees=3.0,
            flywheel_tolerance_rotations_per_second=5.0,
        )

    def _get_shooter_is_within(
        self,
        turret_tolerance_degrees: float,
        hood_tolerance_degrees: float,
        flywheel_tolerance_rotations_per_second: float,
    ) -> bool:
        """Indicates if shooter is within provided tolerances."""
        turret_error = abs(
            self.target_tracker.target_turret_angle_degrees()
            - self.turret.measured_angle_degrees()
        )
        hood_error = abs(
            self.target_tracker.target_hood_angle_degrees()
            - self.hood.measured_angle_degrees()
        )
        flywheel_error = abs(
            self.target_tracker.target_flywheel_speed_rps()
            - self.flywheel.measured_speed_rps()
        )
        return (
            (turret_error <= turret_tolerance_degrees)
            and (hood_error <= hood_tolerance_degrees)
            and (flywheel_error <= flywheel_tolerance_rotations_per_second)
        )

    def _robot_is_moving(self, speed_threshold_mps: float = 0.1) -> bool:
        """Indicates if the robot's linear speed is over the threshold."""
        chassis_speeds: kinematics.ChassisSpeeds = self.drivetrain.robot_speeds()
        robot_speed_mps = math.sqrt(
            (chassis_speeds.vx**2) + (chassis_speeds.vy**2)
        )
        return robot_speed_mps > speed_threshold_mps

    def _log_data(self) -> None:
        self.data_logger.log_boolean(
            "/components/shooter/auto", self._auto, on_change=True
        )
        self.data_logger.log_boolean(
            "/components/shooter/driver_wants_feed",
            self._driver_wants_feed,
            on_change=True,
        )
