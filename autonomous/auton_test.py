
#THIS IS NOT BEING USED AND IS JUT FOR REFERENCE

from magicbot import AutonomousStateMachine, state, timed_state
from subsystem import drivetrain
from phoenix6 import swerve
from common.joystick import DriveCommand

class Auton_Test(AutonomousStateMachine):
    MODE_NAME = "Test"
    DEFAULT = False

    drivetrain: drivetrain.Drivetrain
    # drive_request: swerve.requests.FieldCentric

    @timed_state(first=True, duration=2, next_state="stop")
    def spin(self):
        self.drivetrain.setSpeeds(DriveCommand(0,0,1))

    @state
    def stop(self):
        self.drivetrain.setSpeeds(DriveCommand(0,0,0))
        self.done()