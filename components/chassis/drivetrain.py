import constants
import components.chassis.swervemodule as swervemodule
from wpilib import SmartDashboard 
from wpimath.kinematics import SwerveDriveKinematics
from wpimath.geometry import Translation2d
import wpimath.kinematics

class Drivetrain:
    
    def __init__(self):

        self.kinematics = SwerveDriveKinematics(
            Translation2d(constants.WHEEL_BASE / 2,  constants.TRACK_WIDTH / 2),   # FL
            Translation2d(constants.WHEEL_BASE / 2, -constants.TRACK_WIDTH / 2),   # FR
            Translation2d(-constants.WHEEL_BASE / 2,  constants.TRACK_WIDTH / 2),  # BL
            Translation2d(-constants.WHEEL_BASE / 2, -constants.TRACK_WIDTH / 2),  # BR
        )

        self.front_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FL,
            offset=constants.FL_OFFSET,
            turning_motor_id=constants.STEER_CAN_FL,
            turning_encoder_id=constants.TURN_ENCODER_ID_FL,
            name = "Front Left",
            )
            
        self.front_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FR,
            offset=constants.FR_OFFSET,
            turning_motor_id=constants.STEER_CAN_FR,
            turning_encoder_id=constants.TURN_ENCODER_ID_FR,
            name = "Front Right",
            )
                
        self.back_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BR,
            offset=constants.BR_OFFSET,
            turning_motor_id=constants.STEER_CAN_BR,
            turning_encoder_id=constants.TURN_ENCODER_ID_BR,
            name = "Back Right",
            )

        self.back_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BL,
            offset=constants.BL_OFFSET,
            turning_motor_id=constants.STEER_CAN_BL,
            turning_encoder_id=constants.TURN_ENCODER_ID_BL,
            name = "Back Left",
            )
        
    def drive(self, speeds: wpimath.kinematics.ChassisSpeeds, field_relative: bool = True) -> None:
        pass 

    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the drivetrain based off current state.
        """
        SmartDashboard.putNumber(
            "FL ENCODER ABS", self.front_left.get_encoder_angle_abs()
        )
        SmartDashboard.putNumber(
            "FR ENCODER ABS", self.front_right.get_encoder_angle_abs()
        )
        SmartDashboard.putNumber(
            "BL ENCODER ABS", self.back_left.get_encoder_angle_abs()
        )
        SmartDashboard.putNumber(
            "BR ENCODER ABS", self.back_right.get_encoder_angle_abs()
        )
