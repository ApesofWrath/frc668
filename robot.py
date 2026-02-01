import magicbot
import wpilib
import wpimath
import math
from components.chassis.drivetrain import Drivetrain
from components.hopper import Hopper
from components.indexer import Indexer
import constants

class MyRobot(magicbot.MagicRobot):
    drivetrain: Drivetrain 
    hopper: Hopper 
    indexer: Indexer 

    def createObjects(self):
        """ called on initialization """
        self.main_controller = wpilib.XboxController(0)
        self.operator_controller = wpilib.XboxController(1)

    def disabledInit(self):
        """ called when enter disabled mode """
        
    def disabledPeriodic(self):
        """ called periodically when disabled """

    def autonomousInit(self):
        """ initialization code for auton """

    def autonomousPeriodic(self):
        """ called periodically during auton """

    def teleopInit(self):
        """ initialization code for teleop """

    def teleopPeriodic(self):
        """ called periodically during teleop """
        self.driveWithJoysicks()
        self.controlHopper()
        self.controlIndexer()

    def driveWithJoysicks(self):
        omega = 0
        vx = 0
        vy = 0
        modifier = 1
        if self.main_controller.getLeftBumperButton():
            modifier = 0.13

        if self.main_controller.getRightBumperButton():
            self.drivetrain.gyro.set_yaw(0)

        if self.drivetrain.isManual():
            vx = (
                -filterInput(self.main_controller.getLeftY())
                * constants.MAX_LINEAR_SPEED
                * modifier
            )
            vy = (
                -filterInput(self.main_controller.getLeftX())
                * constants.MAX_LINEAR_SPEED
                * modifier
            )   
            omega = (
                -filterInput(self.main_controller.getRightX())
                * constants.MAX_ROTATION_SPEED
                * modifier
            ) 
        self.drivetrain.vx = vx
        self.drivetrain.vy = vy
        self.drivetrain.omega = omega
    
    def controlHopper(self):
        if self.hopper.isManual():
            if self.operator_controller.getRightBumper():
                self.hopper.motorSpeed = 1
            else:
                self.hopper.motorSpeed = 0
        else:
            #placeholder for auto
            self.hopper.motorSpeed = 0

    def controlIndexer(self):
        if self.indexer.isManual():
            if self.operator_controller.getRightBumper():
                self.indexer.motorSpeed = 1
            else:
                self.indexer.motorSpeed = 0
        else:
            #placeholder for auto
            self.indexer.motorSpeed = 0
        
    
def filterInput(controller_input: float, apply_deadband: bool = True) -> float:
    """
    Filters the controller input by applying a squared scaling and an optional deadband.

    This function squares the input while preserving its sign to provide finer control
    at lower values. If `apply_deadband` is True, it applies a deadband to ignore small
    inputs that may result from controller drift.

    Args:
        controller_input (float): The raw input from the controller, ranging from -1 to 1.
        apply_deadband (bool, optional): Whether to apply a deadband to the input. Defaults to True.

    Returns:
        float: The filtered controller input.
    """
    controller_input_corrected = math.copysign(
        math.pow(controller_input, 2), controller_input
    )

    if apply_deadband:
        return wpimath.applyDeadband(controller_input_corrected, constants.DEADBAND)
    else:
        return controller_input_corrected
        