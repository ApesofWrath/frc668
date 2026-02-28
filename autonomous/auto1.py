from magicbot import AutonomousStateMachine, state
from autonomous import AutoHelper

class Auto1(AutonomousStateMachine):
    MODE_NAME = "auto1"
    DEFAULT = True

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("auto1")
        super().on_enable()

    @state(first=True)
    def follow_path(self,state_tm):
        tick = self.AutoHelper.Tick(state_tm)
        if(tick == 1):
            self.next_state("end")
        
    @state
    def end(self):
        self.AutoHelper.end()
        self.done()