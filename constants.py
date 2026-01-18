# drivetrain front right
DRIVE_CAN_FR = 7
FR_OFFSET = -0.149658203125
STEER_CAN_FR = 8
TURN_ENCODER_ID_FR = 12

# drivetrain front left
DRIVE_CAN_FL = 3
FL_OFFSET = 0.4736328125
STEER_CAN_FL = 4
TURN_ENCODER_ID_FL = 10

# drivetrain back right
DRIVE_CAN_BR = 5
BR_OFFSET = 0.19873046875
STEER_CAN_BR = 6
TURN_ENCODER_ID_BR = 11

# drivetrain back left
DRIVE_CAN_BL = 0
BL_OFFSET = 0.144287109375
STEER_CAN_BL = 1
TURN_ENCODER_ID_BL = 2


# PID. TODO: tune!
DRIVE_P = 2
DRIVE_I = 0
DRIVE_D = 0

TURNING_P = 4
TURNING_I = 0
TURNING_D = 0

WHEEL_BASE = 0.625 # meters
TRACK_WIDTH = 0.625 # meters

WHEEL_RADIUS = 0.045 # meters
GEAR_RATIO = 1/6.75 #TODO: confirm with mechanical design team

MAX_LINEAR_SPEED = 5  # meters per second
MAX_LINEAR_ACCELERATION = 3  # meters per second squared

MAX_ROTATION_SPEED = 7  # radians per second
MAX_ROTATION_ACCELERATION = 1 / 2  # Radians per second squared

MAX_SINGLE_SWERVE_ROTATION_SPEED = 12 # radians per second
MAX_SINGLE_SWERVE_ROTATION_ACCELERATION = 40 # radians per sec squared