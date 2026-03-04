from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class OutpostTrenchNoNeutral(AutonomousStateMachine):
    MODE_NAME = "outpost_trench_no_neutral"
    # DEFAULT = True

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("StartOutpostTrenchGetOutpost",True)
        super().on_enable()

    @state(first=True)
    def get_outpost(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_for_outpost")
    
    @timed_state(duration=4,next_state="get_depot")
    def wait_for_outpost(self):
        pass
    
    @state
    def get_depot(self,state_tm,initial_call):
        if(initial_call):
            self.AutoHelper.reset("OutpostGetDepot")
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.AutoHelper.reset("FarDepotCenterAndShoot")
            self.next_state("center_and_shoot")

    @state
    def center_and_shoot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_while_shooting")

    @timed_state(duration=5,next_state="end")
    def wait_while_shooting(self):
        pass
        
    @state
    def end(self):
        self.AutoHelper.end()
        self.done()