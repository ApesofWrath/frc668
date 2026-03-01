from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class DepotTrenchNoNeutral(AutonomousStateMachine):
    MODE_NAME = "depot_trench_no_neutral"

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("StartDepotTrenchGetDepotAndStartScoring",True)
        super().on_enable()

    @state(first=True)
    def get_depot_and_shoot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_while_shooting")

    @timed_state(duration=5,next_state="end")
    def wait_while_shooting(self):
        pass
            
    @state
    def fifth_state_name(self,state_tm): # SET STATE NAME FOR FIFTH STATE
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("end") # SET NEXT STATE NAME, end TO FINISH
        
    @state
    def end(self):
        self.AutoHelper.end()
        self.done()