import swervemodule 
import constants

class Drivetrain:
    def __init__(self):

        self.front_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FL,
            toffset=constants.FL_OFFSET,
            turning_motor_id=constants.STEER_CAN_FL,
            turning_encoder_id=constants.TURN_ENCODER_ID_FL,
            name ="Front Left",
            )
            
        self.front_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_FR,
            toffset=constants.FR_OFFSET,
            turning_motor_id=constants.STEER_CAN_FR,
            turning_encoder_id=constants.TURN_ENCODER_ID_FR,
            name ="Front Right",
            )
                
        self.back_right = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BR,
            toffset=constants.BR_OFFSET,
            turning_motor_id=constants.STEER_CAN_BR,
            turning_encoder_id=constants.TURN_ENCODER_ID_BR,
            name ="Back Right",
            )

        self.back_left = swervemodule.SwerveModule(
            drive_motor_id=constants.DRIVE_CAN_BL,
            toffset=constants.BL_OFFSET,
            turning_motor_id=constants.STEER_CAN_BL,
            turning_encoder_id=constants.TURN_ENCODER_ID_BL,
            name ="Back Left",
            )