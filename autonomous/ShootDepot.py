from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class ShootDepot(AutonomousStateMachine):
    MODE_NAME = "shoot_from_depot"

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("shootFromDepot",True)
        super().on_enable()

    @state(first=True)
    def shoot(self,state_tm): 
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait") 

    @timed_state(duration=5, next_state="end")
    def wait(self):
        pass

    @state
    def end(self):
        self.AutoHelper.end()
        self.done()