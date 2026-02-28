
#THIS IS NOT BEING USED AND IS JUT FOR REFERENCE

from magicbot import AutonomousStateMachine, state, timed_state
from subsystem import drivetrain
from phoenix6 import swerve

# from pathplannerlib.auto import AutoBuilder
# from pathplannerlib.path import PathPlannerPath


# Pathplanner is going to be ***very*** difficult to use, at least from my testing
class Auton_Test_Pathplanner(AutonomousStateMachine):
    MODE_NAME = "Test2"
    DEFAULT = False

    drivetrain: drivetrain.Drivetrain
    drive_request: swerve.requests.FieldCentric

    @state(first=True)
    def start(self):
        # if not hasattr(self, "command"):
        #     self.command = AutoBuilder.followPath(PathPlannerPath.fromPathFile("pathTest1"))
        #     self.command.schedule()

        # if self.command.isFinished():
        self.next_state("end")


    @state
    def end(self):
        self.done()