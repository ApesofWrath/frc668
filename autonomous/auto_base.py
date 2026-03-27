from typing import Optional

import choreo
import magicbot
import wpilib
from wpimath import geometry

from common import alliance
from subsystem import drivetrain, intake, shooter


class AutoBase(magicbot.AutonomousStateMachine):
    """A base class for autonomous routines.

    This performs the following sequence of events:
    1. Executes the specified trajectory. Shoots on the move if configured.
    2. Comes to a stop and shoots fuel for the specified duration.
    3. Disables the intake and flywheel.
    """

    # The trajectory to follow. Every subclass *must* specifcy this.
    TRAJECTORY_NAME: str = ""
    # If set to True, shoot while executing the trajectory, when we are in our
    # alliance zone.
    SHOOT_ON_THE_MOVE = False
    # If set to a positive value, shoot for this amount of time after the
    # trajectory is executed.
    SHOOT_DURATION_SECONDS: float = 0.0
    # Alliance zone extents.
    BLUE_ZONE_END_X_METERS: float = 5.189
    RED_ZONE_END_X_METERS: float = 11.352

    alliance_fetcher: alliance.AllianceFetcher
    drivetrain: drivetrain.Drivetrain
    intake: intake.Intake
    intake_deployer: intake.IntakeDeployer
    shooter_state_machine: shooter.Shooter

    def setup(self):
        if not self.TRAJECTORY_NAME:
            wpilib.reportError(
                f"{self.__class__.__name__} does not set TRAJECTORY_NAME"
            )
            self._trajectory = None

        try:
            self._trajectory = choreo.load_swerve_trajectory(
                self.TRAJECTORY_NAME
            )
        except ValueError:
            self._trajectory = None
            wpilib.reportError(
                f"Failed to load trajectory '{self.TRAJECTORY_NAME}' - check "
                "deploy/choreo/"
            )

    def getInitialPose(self) -> geometry.Pose2d:
        """Returns the starting pose of the robot for this trajectory."""
        return (
            self._trajectory.get_initial_pose(
                self.alliance_fetcher.isRedAlliance()
            )
            if self._trajectory
            else geometry.Pose2d()
        )

    def on_enable(self) -> None:
        # The intake can remain active for the entire duration of auto.
        self.intake.setActive(True)
        super().on_enable()

    @magicbot.state(first=True)
    def executing_trajectory(self, state_tm) -> None:
        """Executes the trajectory."""
        if self._trajectory is None or state_tm > 20.0:
            self.next_state("shooting_fuel")
            return

        sample: choreo.SwerveSample = self._trajectory.sample_at(
            state_tm, self.alliance_fetcher.isRedAlliance()
        )

        if sample is None:
            self.logger.error(
                f"Failed to get trajectory sample for center_shoot_preload at "
                "time {state_tm}"
            )
            return

        self.drivetrain.followTrajectorySample(sample)

        if state_tm < 1.0 or not self.SHOOT_ON_THE_MOVE:
            return

        # Shoot on the move, when in the alliance zone.
        pose = self.drivetrain.get_robot_pose()
        if self.alliance_fetcher.isRedAlliance():
            if pose.X() > self.RED_ZONE_END_X_METERS:
                # We're in the red alliance zone, shoot at the hub.
                self.shooter_state_machine.setAuto(True)
                self.shooter_state_machine.setDriverWantsFeed(True)
            else:
                # We're in the neutral zone, don't shoot.
                self.shooter_state_machine.setAuto(True)
                self.shooter_state_machine.setDriverWantsFeed(False)
        else:
            if pose.X() < self.BLUE_ZONE_END_X_METERS:
                # We're in the blue alliance zone, shoot at the hub.
                self.shooter_state_machine.setAuto(True)
                self.shooter_state_machine.setDriverWantsFeed(True)
            else:
                # We're in the neutral zone, don't shoot.
                self.shooter_state_machine.setAuto(True)
                self.shooter_state_machine.setDriverWantsFeed(False)

    @magicbot.state
    def shooting_fuel(self, state_tm) -> None:
        """Stops moving and shoots the fuel in the hub."""
        if state_tm >= self.SHOOT_DURATION_SECONDS:
            self.next_state("finished")

        self.drivetrain.stop()
        self.shooter_state_machine.setAuto(True)
        self.shooter_state_machine.setDriverWantsFeed(True)

    @magicbot.state
    def finished(self) -> None:
        """Stops the fuel feed and ends the state machine."""
        self.drivetrain.stop()
        self.intake.setActive(False)
        self.shooter_state_machine.setAuto(True)
        self.shooter_state_machine.setDriverWantsFeed(False)
        self.done()
