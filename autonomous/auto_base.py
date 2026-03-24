import choreo
import magicbot
import wpilib 
from wpimath import geometry

from common import alliance
from subsystem import drivetrain, intake, shooter


class AutoBase(magicbot.AutonomousStateMachine):
    """A base class for autonomous routines.

    This performs the following sequence of events:
    1. Waits for a specified amount of time for the intake to deploy.
    2. Executes the specified trajectory.
    3. Shoots fuel for the specified duration.
    4. Disables the intake and flywheel.
    """

    # The trajectory to follow. Every subclass *must* specifcy this.
    TRAJECTORY_NAME: str = ""
    # The period of time in seconds to wait for the intake to deploy.
    INTAKE_DEPLOY_DURATION_SECONDS: float = 1.0
    # The period of time in seconds to shoot for in the shooting_fuel state.
    SHOOT_DURATION_SECONDS: float = 8.0

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

    @magicbot.state(first=True)
    def deploying_intake(self, state_tm) -> None:
        """Waits for the intake to deploy.

        The deploy is initiated automatically, we just need to wait.
        """
        if (
            self.intake_deployer.hasDeployed()
            or state_tm >= self.INTAKE_DEPLOY_DURATION_SECONDS
        ):
            self.next_state("executing_trajectory")
            return

    @magicbot.state
    def executing_trajectory(self, state_tm) -> None:
        """Executes the trajectory."""
        if (
            self._trajectory is None
            or state_tm > self._trajectory.get_total_time()
        ):
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

    @magicbot.state
    def shooting_fuel(self, state_tm) -> None:
        """Stops moving and shoots the fuel in the hopper."""
        if state_tm >= self.SHOOT_DURATION_SECONDS:
            self.next_state("finished")
            return

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
