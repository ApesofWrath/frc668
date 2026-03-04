from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper

class SmallTest(AutonomousStateMachine):
    MODE_NAME = "small_test"
    DEFAULT = True

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("ShortTest1",True)
        super().on_enable()

    @state(first=True)
    def st1(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait")
    
    @timed_state(duration=3,next_state="set_return_to_start")
    def wait(self):
        self.logger.info(f"dx:{self.AutoHelper.vision.get_robot_pose().X()},dy:{self.AutoHelper.vision.get_robot_pose().Y()},dr:{self.AutoHelper.vision.get_robot_pose().rotation().radians()}")
        pass
    
    @state
    def set_return_to_start(self):
        self.AutoHelper.reset("ShortTest1")
        self.next_state("return_to_start")

    @state
    def return_to_start(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("end")
        
    @state
    def end(self):
        self.AutoHelper.end()
        self.done()