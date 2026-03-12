from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class DepotTrench_GetAndShoot(AutonomousStateMachine):
    MODE_NAME = "depottrench_depot_shoot"

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("testpath",True)#Depot_GetDepotAndShoot
        super().on_enable()

    @state(first=True)
    def get_depot_and_shoot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.AutoHelper.stop_moving()
            self.next_state("shoot_until_end")
    
    @timed_state(duration=8,next_state="end")
    def shoot_until_end(self):
        pass
        
    @state
    def end(self):
        self.AutoHelper.end()
        self.done()