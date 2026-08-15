###FRC Team 668 Robot System Overview
##Our robot is divided into three core subsystems: Drivetrain, Intake, and Shooter. Below is a detailed ##breakdown of the components that make up each subsystem.

#Drivetrain
Responsible for field mobility and robot positioning. It consists of two main components:

Drive Mechanics: Provides direct translation and rotational control over the robot, including speed management and orientation adjustments.

Vision System: Utilizes four Limelight cameras to scan AprilTags around the field. It processes this data through a Kalman filter to accurately calculate the robot's real-time position relative to the field.

#Intake
Responsible for acquiring game pieces (fuel/balls) and loading them into the hopper. It consists of two main components:

Intake Mechanism: Manages the active intake roller, driving it at calibrated speeds when commanded.

Intake Deployer: Actuates the intake assembly, lowering it from inside the hopper frame to the ground level.

#Shooter
Responsible for feeding, aiming, and launching game pieces into the hub. It consists of five main components:

Hopper: Funnels incoming game pieces from the intake into the indexer mechanism.

Indexer: Elevates game pieces from the hopper up toward the turret assembly.

Turret: Controls horizontal targeting angle, enabling the robot to score in the hub from anywhere on the alliance side of the field.

Flywheel: Accelerates game pieces to control shot velocity and adjust distance/range.

Hood: Controls vertical trajectory and launch angle for accurate scoring.