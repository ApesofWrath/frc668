import math
import wpimath
import constants
from phoenix6 import hardware, swerve
from wpilib import SmartDashboard 
from wpimath.geometry import Translation2d, Rotation2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds, SwerveModuleState

class Drivetrain(swerve.SwerveDrivetrain): #<--- Defines a new class called 'Drivetrain'    
    def __init__(self): #<--- Calls to initialize drivetrain components
        super().__init__(hardware.TalonFX, hardware.TalonFX, hardware.CANcoder, constants.TunerConstants.drivetrain_constants, [constants.TunerConstants.front_left, constants.TunerConstants.front_right, constants.TunerConstants.back_left, constants.TunerConstants.back_right])

    def execute(self)->None:
        pass 

    def isManual(self):
        return True