import constants
import components.chassis.swervemodule as swervemodule
from wpilib import SmartDashboard 

class Drivetrain:

    SmartDashboard.putNumber("FL Offset", constants.FL_OFFSET)
    SmartDashboard.putNumber("FR Offset", constants.FR_OFFSET)
    SmartDashboard.putNumber("BL Offset", constants.BL_OFFSET)
    SmartDashboard.putNumber("BR Offset", constants.BR_OFFSET)
    
    def __init__(self):

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
        
    def execute(self) -> None:
        """
        Called periodically, runs all necessary logic to operate the drivetrain based off current state.
        """