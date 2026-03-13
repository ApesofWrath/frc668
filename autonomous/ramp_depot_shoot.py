from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class RampDepotShoot(AutonomousStateMachine):
    MODE_NAME = "ramp_depot_side_shoot"

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("RampDepot_GetDepotAndShoot_copy1",True)#testpath
        super().on_enable()
    
    @timed_state(first=True,duration=1,next_state="get_depot_and_shoot")
    def wait_for_intake(self):
        pass

    @state()
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