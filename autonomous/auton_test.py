
#THIS IS NOT BEING USED AND IS JUT FOR REFERENCE

from magicbot import AutonomousStateMachine, state, timed_state
from subsystem import drivetrain
from phoenix6 import swerve

class Auton_Test(AutonomousStateMachine):
    MODE_NAME = "Test"
    DEFAULT = False

    drivetrain: drivetrain.Drivetrain
    drive_request: swerve.requests.FieldCentric

    @timed_state(first=True, duration=2, next_state="stop")
    def spin(self):
        self.drivetrain.set_control(
            self.drive_request.with_velocity_x(0)
            .with_velocity_y(0)
            .with_rotational_rate(0.6)
        )

    @state
    def stop(self):
        self.drivetrain.set_control(
            self.drive_request.with_velocity_x(0)
            .with_velocity_y(0)
            .with_rotational_rate(0)
        )
        self.done()