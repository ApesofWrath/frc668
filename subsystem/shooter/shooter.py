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
        self.target_tracker.trackPosition(self._auto)
        self.target_tracker.trackSpeed(False)
        self.target_tracker.setTargetFlywheelSpeedRps(
            self.robot_constants.shooter.flywheel.default_speed_rps
        )

        # Don't feed fuel.
        self.hopper.setEnabled(False)
        self.indexer.setEnabled(False)

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
        if self._shooterIsReady():
            self.next_state_now("shooting")

        # The driver wants to shoot but the shooter isn't ready or the robot is
        # moving. We want to continue tracking the target, but don't feed fuel.
        self.target_tracker.trackPosition(self._auto)
        self.target_tracker.trackSpeed(self._auto)

        self.hopper.setEnabled(False)
        self.indexer.setEnabled(False)

    @magicbot.state
    def shooting(self):
        """Actively shooting.

        Continue tracking the target, and feed fuel.
        """
        if not self._driver_wants_feed:
            self.next_state("idling")

        if not self._shooterIsReady():
            self.next_state("targeting")

        # Fully track the target.
        self.target_tracker.trackPosition(self._auto)
        self.target_tracker.trackSpeed(self._auto)

        # Feed fuel.
        self.hopper.setEnabled(True)
        self.indexer.setEnabled(True)

    def setDriverWantsFeed(self, value: bool) -> None:
        self._driver_wants_feed = value

    def setAuto(self, value: bool) -> None:
        self._auto = value

    def _shooterIsReady(self) -> bool:
        """Indicates if shooter components are close enough to their targets."""
        return self._getShooterIsWithin(
            turret_tolerance_degrees=3.0,
            hood_tolerance_degrees=1.5,
            flywheel_tolerance_rotations_per_second=3.0,
        )

    def _getShooterIsWithin(
        self,
        turret_tolerance_degrees: float,
        hood_tolerance_degrees: float,
        flywheel_tolerance_rotations_per_second: float,
    ) -> bool:
        """Indicates if shooter is within provided tolerances."""
        turret_error = abs(
            self.target_tracker.targetTurretAngleDegrees()
            - self.turret.measuredAngleDegrees()
        )
        hood_error = abs(
            self.target_tracker.targetHoodAngleDegrees()
            - self.hood.measuredAngleDegrees()
        )
        flywheel_error = abs(
            self.target_tracker.targetFlywheelSpeedRps()
            - self.flywheel.measuredSpeedRps()
        )
        return (
            (turret_error <= turret_tolerance_degrees)
            and (hood_error <= hood_tolerance_degrees)
            and (flywheel_error <= flywheel_tolerance_rotations_per_second)
        )

    def _robotIsMoving(self, speed_threshold_mps: float = 0.1) -> bool:
        """Indicates if the robot's linear speed is over the threshold."""
        chassis_speeds: kinematics.ChassisSpeeds = self.drivetrain.robotSpeeds()
        robot_speed_mps = math.sqrt(
            (chassis_speeds.vx**2) + (chassis_speeds.vy**2)
        )
        return robot_speed_mps > speed_threshold_mps

    def _logData(self) -> None:
        self.data_logger.logBoolean(
            "/components/shooter/auto", self._auto, on_change=True
        )
        self.data_logger.logBoolean(
            "/components/shooter/driver_wants_feed",
            self._driver_wants_feed,
            on_change=True,
        )
