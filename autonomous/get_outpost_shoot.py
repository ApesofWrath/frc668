import choreo
from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake 

class BumpOutpostShoot(AutonomousStateMachine):
    """Start on the outpose side of the ramp, drive  to outpost, wait for human to load fuel, then shoot."""
    MODE_NAME = "get_outpost_shoot"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer

    def __init__(self):
        self.trajectory = choreo.load_swerve_trajectory("bump_to_outpost")

    def on_enable(self):
        self.AutoHelper.reset("bump_to_outpost",True)
        super().on_enable()

    @timed_state(first=True, duration=1.0, next_state="move_to_outpost")
    def wait_for_intake(self):
        if self.intake_deployer._deployed: 
            self.next_state("move_to_outpost")

    @state()
    def move_to_outpost(self,state_tm): 
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("wait_for_loading") 

    @timed_state(duration=4, next_state="move_to_shoot")
    def wait_for_loading(self):
        self.AutoHelper.stop_moving()
        
    @state()
    def move_to_shoot(self, state_tm, initial_call):
        if initial_call:
            self.AutoHelper.reset("outpost_to_shoot")
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