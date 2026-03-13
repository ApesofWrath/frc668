from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake

class ShootDepotRot(AutonomousStateMachine):    
    MODE_NAME = "test_shoot_from_depot_rot"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer 

    def on_enable(self):
        self.AutoHelper.reset("testShootFromDepotRot",True)
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