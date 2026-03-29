import choreo
from magicbot import AutonomousStateMachine, state, timed_state
from autonomous import AutoHelper
from subsystem import intake 

class DepotNeutralZoneShoot(AutonomousStateMachine):
    """Start on the depot side of the ramp, drive to the neutral zone, intake fuel, come back and shoot."""
    MODE_NAME = "get_neutral_zone_shoot_starting_depot_side"

    AutoHelper: AutoHelper.AutoHelper
    intake_deployer: intake.IntakeDeployer

    def __init__(self):
        self.trajectory = choreo.load_swerve_trajectory("DepotSideBump_IntakeNeutralZone_Shoot")

    def on_enable(self):
        self.AutoHelper.reset("DepotSideBump_IntakeNeutralZone_Shoot",True)
        super().on_enable()
    
    @timed_state(first=True,duration=1,next_state="move_and_intake")
    def wait_for_intake(self):
        if self.intake_deployer._deployed: 
            self.next_state("move_and_intake")

    @state()
    def move_and_intake(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("shoot")
    
    @timed_state(duration=8,next_state="end")
    def shoot(self):
        self.AutoHelper.stop_moving()
        
    @state()
    def end(self):
        self.AutoHelper.shooter_state_machine.setDriverWantsFeed(False)
        self.AutoHelper.end()
        self.done()