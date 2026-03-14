from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake

class ShootDepotBumps(AutonomousStateMachine):
    """Start at the depot side of the ramp. Drive over the bumps to the neutral zone. No intake in neutral zone."""
    MODE_NAME = "from_depot_drive_over_bumps"
    # This one wasn't actually able to shoot according to our tests. 
    # Try "shoot_preload_drive_neutral_zone" (not tested though) to 
    # shoot the preloaded fuel before driving to neutral zone.

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer 

    def on_enable(self):
        self.AutoHelper.reset("ShootFromDepot_DriveOverBumps",True)
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
            self.AutoHelper.reset("ShootFromDepot_DriveOverBumps")
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("end")

    @state()
    def end(self):
        self.AutoHelper.shooter_state_machine.setDriverWantsFeed(False)
        self.AutoHelper.end()
        self.done()