from magicbot import state, StateMachine
from subsystem import shooter, drivetrain
import constants
import math


class ShooterStateMachine(StateMachine):

    is_shooting = False
    should_idle = False
    should_exit = False
    disable_shooting_while_moving = True

    turret: shooter.Turret
    hood: shooter.Hood
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer
    drivetrain: drivetrain.Drivetrain
    hub_tracker: shooter.HubTracker
    robot_constants: constants.RobotConstants

    @state(first=True)
    def targeting(self, initial_call):
        """Attempting to get to target postion"""
        self.hub_tracker.setEnabled(True)
        if (
            self._get_shooter_state_is_target_state_default()
            and self.is_shooting
            and not self.is_drivetrain_moving()
        ):
            self.next_state_now("shooting")

        if self.should_idle:
            self.next_state("idling")

        if self.should_exit:
            self.next_state("exit")

        if (
            initial_call
            or self.hopper.get_enabled()
            or self.indexer.get_enabled()
        ):
            self.hopper.setEnabled(False)
            self.indexer.setEnabled(False)
        self.logger.info(self._shooter_state_vs_target_state(3, 1.5, 3))

    @state
    def idling(self):
        """Waiting to be in a place where we should prepare to shoot"""
        self.hub_tracker.setEnabled(False)
        self.flywheel.setTargetRps(
            self.robot_constants.shooter.flywheel.default_speed_rps
        )
        self.turret.setPosition(
            self.hub_tracker.get_target_turret_angle_degrees()
        )

        if not self.should_idle:
            self.next_state("targeting")

        if self.should_exit:
            self.next_state("exit")

    @state
    def shooting(self, initial_call):
        """Actively shooting"""
        self.hub_tracker.setEnabled(True)
        if (
            not self.is_shooting
            or not self._get_shooter_state_is_target_state_default()
            or self.is_drivetrain_moving()
        ):
            self.next_state("targeting")
            return

        if (
            initial_call
            or not self.hopper.get_enabled()
            or not self.indexer.get_enabled()
        ):
            self.hopper.setEnabled(True)
            self.indexer.setEnabled(True)

        if self.should_idle:
            self.next_state("idling")

    @state
    def exiting(self):
        """Stoping the shooter and going back to the current zeros"""
        self.hub_tracker.setEnabled(False)
        self.turret.setPosition(0)
        self.hood.setPosition(0)
        self.flywheel.setTargetRps(
            self.robot_constants.shooter.flywheel.default_speed_rps
        )

    def _get_shooter_state_is_target_state_default(self) -> bool:
        return self._get_shooter_state_is_target_state(3, 1.5, 3)

    def _get_shooter_state_is_target_state(
        self,
        turret_tolerance_degrees: float,
        hood_tolerance_degrees: float,
        flywheel_tolerance_rotations_per_second: float,
    ) -> bool:
        turret_error = abs(
            self.hub_tracker.get_predictive_turret_target_angle_degrees()
            - self.turret.get_turret_angle()
        )
        hood_error = abs(
            self.hub_tracker.get_target_hood_angle_degrees()
            - self.hood.get_measured_angle_degrees()
        )
        flywheel_error = abs(
            self.hub_tracker.get_target_flywheel_speed_rps()
            - self.flywheel.flywheel_encoder.get_velocity().value
        )
        return (
            (turret_error <= turret_tolerance_degrees)
            & (hood_error <= hood_tolerance_degrees)
            & (flywheel_error <= flywheel_tolerance_rotations_per_second)
        )

    def _shooter_state_vs_target_state(
        self,
        turret_tolerance_degrees: float,
        hood_tolerance_degrees: float,
        flywheel_tolerance_rotations_per_second: float,
    ) -> str:
        turret_error = abs(
            self.hub_tracker.get_predictive_turret_target_angle_degrees()
            - self.turret.get_turret_angle()
        )
        hood_error = abs(
            self.hub_tracker.get_target_hood_angle_degrees()
            - self.hood.get_measured_angle_degrees()
        )
        flywheel_error = abs(
            self.hub_tracker.get_target_flywheel_speed_rps()
            - self.flywheel.flywheel_encoder.get_velocity().value
        )
        te = f"{turret_error}, {"pass" if (turret_error <= turret_tolerance_degrees) else "fail"}"
        he = f"{hood_error}, {"pass" if (hood_error <= hood_tolerance_degrees) else "fail"}"
        fe = f"{flywheel_error}, {"pass" if (flywheel_error <= flywheel_tolerance_rotations_per_second) else "fail"}"
        return (
            f"Turret error is {te}. Hood error is {he}. Flywheel error is {fe}."
        )

    def is_drivetrain_moving(self) -> bool:
        return self.get_total_drivetrain_speed() > 0.1

    def get_total_drivetrain_speed(self) -> float:
        return math.sqrt(
            (self.drivetrain.get_robot_speed().vx ** 2)
            + (self.drivetrain.get_robot_speed().vy ** 2)
        )
