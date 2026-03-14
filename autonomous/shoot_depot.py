from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake

class ShootDepot(AutonomousStateMachine):    
    """Start at the depot side of the ramp and shoot the preloaded fuel."""
    MODE_NAME = "shoot_from_depot"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer 

    def on_enable(self):
        self.AutoHelper.reset("ShootFromDepot",True)
        super().on_enable()

    @timed_state(first=True, duration=1.0, next_state="move")
    def deployIntake(self):
        if self.intake_deployer._deployed: 
            self.next_state("move")

    @state()
    def move(self,state_tm): 
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