import magicbot
import wpilib
import wpimath
from components.chassis.drivetrain import Drivetrain

class MyRobot(magicbot.MagicRobot):
    drivetrain: Drivetrain 

    def createObjects(self):
        """ called on initialization """
        self.main_controller = wpilib.XboxController(0)

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
        # TODO: button bindings