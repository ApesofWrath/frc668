from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake

class ShootPreloadDepotBumps(AutonomousStateMachine):
    """Start at the depot side of the ramp. Shoot the preloaded fuel, then drive over the bumps to the neutral zone. No intake in neutral zone."""
    MODE_NAME = "shoot_preloaded_from_depot_drive_over_bumps"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer 

    def on_enable(self):
        self.AutoHelper.reset("depot_to_shot",True)
        super().on_enable()

    @timed_state(first=True, duration=1.0, next_state="move_to_shoot")
    def wait_for_intake(self):
        if self.intake_deployer._deployed: 
            self.next_state("move_to_shoot")

    @state()
    def move_to_shoot(self,state_tm): 
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("shoot") 

    @timed_state(duration=5, next_state="drive_over_bumps")
    def shoot(self):
        self.AutoHelper.stop_moving()
        
    @state()
    def drive_over_bumps(self, state_tm, initial_call):
        if initial_call:
            self.AutoHelper.reset("shoot_to_bump_enter_neutral_zone")
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("end")

    @state()
    def end(self):
        self.AutoHelper.shooter_state_machine.setDriverWantsFeed(False)
        self.AutoHelper.end()
        self.done()