import choreo
from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake

class ShootCenter(AutonomousStateMachine):
    """Start at the center, shoot preloaded fuel."""
    MODE_NAME = "shoot_from_center"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer

    def __init__(self):
        self.trajectory = choreo.load_swerve_trajectory("ShootFromCenter")

    def on_enable(self):
        self.AutoHelper.reset("ShootFromCenter",True)
        super().on_enable()

    @timed_state(first=True, duration=1.0, next_state="moveToCenter")
    def deployIntake(self):
        if self.intake_deployer._deployed: 
            self.next_state("moveToCenter")

    @state()
    def moveToCenter(self,state_tm): 
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("shoot") 

    @timed_state(duration=8, next_state="end")
    def shoot(self):
        self.AutoHelper.stop_moving()

    @state()
    def end(self):
        self.AutoHelper.shooter_state_machine.setDriverWantsFeed(False)
        self.AutoHelper.end()
        self.done()