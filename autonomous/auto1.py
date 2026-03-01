from magicbot import AutonomousStateMachine, state
from autonomous import AutoHelper

class Auto1(AutonomousStateMachine):
    MODE_NAME = "auto1"
    DEFAULT = True

    AutoHelper: AutoHelper.AutoHelper

    def on_enable(self):
        self.AutoHelper.reset("StartDepotTrenchGetDepotAndShoot",True)
        super().on_enable()

    @state(first=True)
    def collect_depot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.AutoHelper.reset("DepotRampOutAndCollect")
            self.next_state("collect_neutral_from_depot")

    @state
    def collect_neutral_from_depot(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.AutoHelper.reset("NeutralOutpostFeedAndCollect")
            self.next_state("feed_from_outpost_neutral")

    @state
    def feed_from_outpost_neutral(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.AutoHelper.reset("NeutralDepotBumpAndScore")
            self.next_state("score_from_depot_bump")
            
    @state
    def score_from_depot_bump(self,state_tm):
        tick_result = self.AutoHelper.Tick(state_tm)
        if(tick_result == 1):
            self.next_state("end")
        
    @state
    def end(self):
        self.AutoHelper.end()
        self.done()