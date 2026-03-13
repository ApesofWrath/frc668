from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake 

class BumpDepotShoot(AutonomousStateMachine):
    MODE_NAME = "get_depot_shoot"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer

    def on_enable(self):
        self.AutoHelper.reset("DepotSideBump_GetDepotAndShoot",True)
        super().on_enable()
    
    @timed_state(first=True,duration=1,next_state="get_depot_and_shoot")
    def wait_for_intake(self):
        if self.intake_deployer._deployed: 
            self.next_state("get_depot_and_shoot")

    @state()
    def get_depot_and_shoot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("shoot_until_end")
    
    @timed_state(duration=8,next_state="end")
    def shoot_until_end(self):
        self.AutoHelper.stop_moving()
        
    @state()
    def end(self):
        self.AutoHelper.shooter_state_machine.setDriverWantsFeed(False)
        self.AutoHelper.end()
        self.done()