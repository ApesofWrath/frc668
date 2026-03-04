from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class DepotTrenchNoNeutral(AutonomousStateMachine):
    MODE_NAME = "depot_trench_no_neutral"
    DEFAULT = False

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("StartDepotTrenchGetDepotAndStartScoring",True)
        super().on_enable()

    @state(first=True)
    def get_depot_and_shoot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_while_shooting")

    @timed_state(duration=5,next_state="get_outpost")
    def wait_while_shooting(self):
        pass
            
    @state
    def get_outpost(self,state_tm,initial_call):
        if(initial_call):
            self.AutoHelper.reset("CenterGetOutpost")
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_for_outpost")
    
    @timed_state(duration=4,next_state="recenter_and_shoot")
    def wait_for_outpost(self):
        pass
    
    @state
    def recenter_and_shoot(self,state_tm,initial_call):
        if(initial_call):
            self.AutoHelper.reset("OutpostCenterAndShoot")
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_while_shooting_2")

    @timed_state(duration=5,next_state="end")
    def wait_while_shooting_2(self):
        pass

    @state
    def end(self):
        self.AutoHelper.end()
        self.done()