from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake

class ShootDepotBumps(AutonomousStateMachine):
    MODE_NAME = "shoot_from_depot_bumps"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer 

    def on_enable(self):
        self.AutoHelper.reset("shootFromDepotDriveBumps",True)
        super().on_enable()

    @timed_state(first=True, duration=1.0, next_state="move")
    def wait_for_intake(self):
        if self.intake_deployer._deployed: 
            self.next_state("move")

    @state()
    def move(self,state_tm): 
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("shoot") 

    @timed_state(duration=8, next_state="drive_over_bumps")
    def shoot(self):
        self.AutoHelper.stop_moving()
        
    @state()
    def drive_over_bumps(self, state_tm, initial_call):
        if initial_call:
            self.AutoHelper.reset("shootFromCenter")
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("end")

    @state()
    def end(self):
        self.AutoHelper.shooter_state_machine.setDriverWantsFeed(False)
        self.AutoHelper.end()
        self.done()