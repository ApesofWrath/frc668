import math

import magicbot

import constants
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
    hub_tracker: shooter.HubTracker
    allience_tracker: shooter.AllienceTracker
    robot_constants: constants.RobotConstants

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
        self._setTracker(self.allience_tracker.getInAllienceZone(),self._auto,False)
        self.flywheel.setTargetRps(
            self.robot_constants.shooter.flywheel.default_speed_rps
        )

        # Don't feed fuel.
        self.hopper.setEnabled(False)
        self.indexer.setEnabled(False)

    @magicbot.state
    def targeting(self):
        """Attempting to get to target state.

        This means the hub and turret must be close to their target angles, the
        flywheel must be close to its target speed, and the robot is stationary.
        """
        # First, check if the driver wants to shoot. If not, idle to save power.
        if not self._driver_wants_feed:
            self.next_state("idling")

        # Then, check if we are ready to shoot.
        if self._shooterIsReady() and not self._robotIsMoving():
            self.next_state_now("shooting")

        # The driver wants to shoot but the shooter isn't ready or the robot is
        # moving. We want to continue tracking the hub, but don't feed fuel.
        self._setTracker(self.allience_tracker.getInAllienceZone(),self._auto,self._auto)

        self.hopper.setEnabled(False)
        self.indexer.setEnabled(False)

    @magicbot.state
    def shooting(self):
        """Actively shooting.

        Continue tracking the hub, and feed fuel.
        """
        if not self._driver_wants_feed:
            self.next_state("idling")

        if not self._shooterIsReady() or self._robotIsMoving():
            self.next_state("targeting")

        # Fully track the hub.
        self._setTracker(self.allience_tracker.getInAllienceZone(),self._auto,self._auto)

        self.allience_tracker.setTrack(self._auto,self._auto)
        if self.allience_tracker.getInAllienceZone():
            self.hub_tracker.trackPosition(self._auto)
            self.hub_tracker.trackSpeed(self._auto)

        # Feed fuel.
        self.hopper.setEnabled(True)
        self.indexer.setEnabled(True)

    def setDriverWantsFeed(self, value: bool) -> None:
        self._driver_wants_feed = value

    def setAuto(self, value: bool) -> None:
        self._auto = value

    def _shooterIsReady(self) -> bool:
        """Indicates if shooter components are close enough to their targets."""
        if self.allience_tracker.getInAllienceZone():
            return self._getShooterIsWithin(
                turret_tolerance_degrees=3.0,
                hood_tolerance_degrees=1.5,
                flywheel_tolerance_rotations_per_second=3.0,
            )
        else:
            return self._getShooterIsWithin(
                turret_tolerance_degrees=10,
                hood_tolerance_degrees=5,
                flywheel_tolerance_rotations_per_second=15 #it doesn't need to be accurate, it needs to be continuous
            )

    def _getShooterIsWithin(
        self,
        turret_tolerance_degrees: float,
        hood_tolerance_degrees: float,
        flywheel_tolerance_rotations_per_second: float,
        hubtracker: bool = True #True if hubtracker, false if alliencetracker
    ) -> bool:
        """Indicates if shooter is within provided tolerances."""
        if hubtracker:
            turret_error = abs(
                self.hub_tracker.get_target_turret_angle_degrees()
                - self.turret.get_measured_angle_degrees()
            )
            hood_error = abs(
                self.hub_tracker.get_target_hood_angle_degrees()
                - self.hood.get_measured_angle_degrees()
            )
            flywheel_error = abs(
                self.hub_tracker.get_target_flywheel_speed_rps()
                - self.flywheel.get_measured_speed_rps()
            )
        else:
            turret_error = abs(
                self.allience_tracker.get_target_turret_angle_degrees()
                - self.turret.get_measured_angle_degrees()
            )
            hood_error = abs(
                self.allience_tracker.get_target_hood_angle_degrees()
                - self.hood.get_measured_angle_degrees()
            )
            flywheel_error = abs(
                self.allience_tracker.get_target_flywheel_speed_rps()
                - self.flywheel.get_measured_speed_rps()
            )
        return (
            (turret_error <= turret_tolerance_degrees)
            and (hood_error <= hood_tolerance_degrees)
            and (flywheel_error <= flywheel_tolerance_rotations_per_second)
        )

    def _robotIsMoving(self, speed_threshold_mps: float = 0.1) -> bool:
        """Indicates if the robot's linear speed is over the threshold."""
        chassis_speeds: swerve.ChassisSpeeds = self.drivetrain.get_robot_speed()
        robot_speed_mps = math.sqrt(
            (chassis_speeds.vx**2) + (chassis_speeds.vy**2)
        )
        return robot_speed_mps > speed_threshold_mps

    def _setTracker(self, useHubtracker, trackPosition, trackSpeed):
        if useHubtracker:
            self.hub_tracker.setEnabled(True)
            self.allience_tracker.setEnabled(False)
            self.hub_tracker.trackPosition(trackPosition)
            self.hub_tracker.trackSpeed(trackSpeed)
        else:
            self.allience_tracker.setEnabled(True)
            self.hub_tracker.setEnabled(False)
            self.allience_tracker.setTrack(trackSpeed,trackPosition)