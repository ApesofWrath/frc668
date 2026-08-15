# frc668


Our robot is broken into three subsystems: drivetrain, intake, and shooter.

Within each subsystem there are components

Drivetrain:

This subsystem is responsible for the movement of the robot on the field
It is made up of mainly two components being the drivetrain and vision

The drivetrain gives direct control over what the drivetrain does such as setting speeds and changing orientation

Vision uses four Limelights to scan Apriltags around the field and using a Kalman filter take this data to accurately  determine the position of the robot relative to the field.  


Intake:

This subsystem is responsible for picking up fuels into the hopper
It is made up of two components being the intake and the intake deployer

The intake controls the intake roller when commanded will run the roller at a certain speed

The intake deployer is responsible for dropping the intake from inside the hopper to touching the ground


Shooter:

This subsystem is responsible for shooting the balls from the hopper into hub primarily
It is made up of five components

The flywheel is responsible shooting the balls at varying ranges.

The hood is responsible for adjusting the angle at which the balls are shot

The hopper is responsible for funneling the balls to the indexer which through there gets shot

The indexer is responsible for moving the balls up to the turret where it eventually gets shot.

The turret is responsible for changing the angle at which the balls are shot at so the robot can shoot at the hub from any where on the alliance side of the field.

