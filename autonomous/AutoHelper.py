from magicbot import AutonomousStateMachine, state, timed_state
from subsystem import drivetrain, shooter, intake
from phoenix6 import swerve
import choreo
from wpimath.geometry import Pose2d
from magicbot import AutonomousStateMachine, state
import wpilib

class AutoHelper():
    drivetrain: drivetrain.Drivetrain
    flywheel: shooter.Flywheel
    turret: shooter.Turret
    hood: shooter.Hood
    indexer: shooter.Indexer
    hopper: shooter.Hopper
    intake: intake.Intake
    
    drive_request: swerve.requests.FieldCentric

    def __init__(self) -> None:
        pass

    def reset(self,path:str,reset_rot = False):
        self.trajectory = choreo.load_swerve_trajectory(path)
        initial_pose = self.trajectory.get_initial_pose()
        if initial_pose is None:
            self.logger.error("Choreo trajetory initial_pose is None")
            return
        if(reset_rot):
            self.drivetrain.reset_odometry(initial_pose)
        self.traj_time = 0.0
        self.triggered_events = []

    def Tick(self,state_tm):
        """
        Follows the trajectory provided, and executes events along the way

        :return: an int representing if the trajectory has finished, with 0 being 'in progress' and 1 being 'done'
        :rtype: int
        """
        self.traj_time = state_tm
        
        sample = self.trajectory.sample_at(self.traj_time)
        if sample is None:
            self.logger.error(f"Failed to get trajectory sample at time {self.traj_time}")
            return
        
        # this control method should probably get improved to use more of the sample's info at some point
        self.drivetrain.set_control(
            self.drive_request.with_velocity_x(sample.vx)
            .with_velocity_y(sample.vy)
            .with_rotational_rate(sample.omega)
        )


        for i in range(len(self.trajectory.events)):
            e = self.trajectory.events[i]
            if((e.timestamp <= self.traj_time) & (self.triggered_events.count(i) == 0)):
                self.triggered_events.append(i)
                self.handle_event(e.event)

        if self.traj_time > self.trajectory.get_total_time():
            return 1
        return 0


    def end(self):
        self.drivetrain.set_control(
            self.drive_request.with_velocity_x(0)
            .with_velocity_y(0)
            .with_rotational_rate(0)
        )

    
    def handle_event(self, event):
        func = event.split(":")[0]
        val = None
        if(event.count(":") != 0):
            val = event.split(":")[1]
        match func:
            case "flywheel.speed":
                if val is None: self._raise_value_not_specified(event)
                self.flywheel.setTargetRps(float(val))
            case "turret.position":
                if val is None: self._raise_value_not_specified(event)
                self.turret.setPosition(float(val))
            case "hood.position":
                if val is None: self._raise_value_not_specified(event)
                self.hood.setPosition(float(val))
            case "shooter.enable":
                self.hopper.setEnabled(True)
                self.indexer.setEnabled(True)
            case "shooter.disable":
                self.hopper.setEnabled(False)
                self.indexer.setEnabled(False)
            case "intake.enable":
                self.intake.setMotorSpeed(1.0)
            case "intake.disable":
                self.intake.setMotorSpeed(0.0)
            case "logger.log":
                self.logger.info(val)
            
    

    def _raise_value_not_specified(self, event):
        raise Exception(f"\nValue not specified in event '{event}'. \n         (use something more like '{event}:10' instead)")
    
    def execute(self):
        pass
