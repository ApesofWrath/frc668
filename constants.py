# FL and BR turn encoders dont seem to exist

# drivetrain front right
DRIVE_CAN_FR = 7
FR_OFFSET = 0
STEER_CAN_FR = 8
TURN_ENCODER_ID_FR = 12

# drivetrain front left
DRIVE_CAN_FL = 3
FL_OFFSET = 0
STEER_CAN_FL = 4
#TURN_ENCODER_ID_FL = 0

# drivetrain back right
DRIVE_CAN_BR = 5
BR_OFFSET = 0
STEER_CAN_BR = 6
#TURN_ENCODER_ID_BR = 0

# drivetrain back left
DRIVE_CAN_BL = 0
BL_OFFSET = 0
STEER_CAN_BL = 1
TURN_ENCODER_ID_BL = 2


# PID 
DRIVE_P = 0
DRIVE_I = 0
DRIVE_D = 0

TURNING_P = 0
TURNING_I = 0
TURNING_D = 0


WHEEL_RADIUS = 0
GEAR_RATIO = 0
TURN_ENCODER_TICKS = 0

MAX_LINEAR_SPEED = 5  # meters per sec
MAX_LINEAR_ACCELERATION = 3  # meters per second squared

MAX_ROTATION_SPEED = 7  # radians per sec
MAX_ROTATION_ACCELERATION = 1 / 2  # Radians per second squared

MAX_SINGLE_SWERVE_ROTATION_SPEED = 9999  # degrees per sec
MAX_SINGLE_SWERVE_ROTATION_ACCELERATION = 9999  # degrees per second squared