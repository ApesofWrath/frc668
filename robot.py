import magicbot
import wpilib
from components import drivetrain

class MyRobot(magicbot.MagicRobot):
    drivetrain: Drivetrain 
    
    def createObjects(self):
        return super().createObjects()
    
    def autonomousInit(self):
        return super().autonomousInit()
    
    def autonomousPeriodic(self):
        return super().autonomousPeriodic()
    
    def teleopInit(self):
        return super().teleopInit()
    
    def teleopPeriodic(self):
        return super().teleopPeriodic()
    