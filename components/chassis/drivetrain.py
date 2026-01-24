import math
import wpimath

from phoenix6 import hardware, swerve

from components.chassis import constants


class Drivetrain(swerve.SwerveDrivetrain):
    def __init__(self):
        super().__init__(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            constants.TunerConstants.drivetrain_constants,
            [
                constants.TunerConstants.front_left,
                constants.TunerConstants.front_right,
                constants.TunerConstants.back_left,
                constants.TunerConstants.back_right,
            ],
        )

    def execute(self) -> None:
        pass

    def isManual(self):
        return True


    # Need to update this method to use vision class instead of diectly
    def field_location(self) -> geometry.Pose2d | None:
    """
    Main method for determining field-relative pose using MegaTag2.
    """
    is_valid, reason = self.mega_tag2_logic(self.l1)
    if is_valid:
        dist = self.mega_tag2.avg_tag_dist 
        stdDevs = (
            0.1 + dist * 0.05,
            0.1 + dist * 0.05, 
            0.2 + dist * 0.1       
        )
        self.drivetrain.addVisionMeasurement(
            self.mega_tag2.pose,
            self.mega_tag2.timestamp_seconds * units.second,
            stdDevs
        )
        return self.drivetrain.get_state().pose

    

        def start_match (self) -> None:
        """
        Resets the pigeon yaw at the start of the match based on alliance color.
        This is necessary to ensure that the robot's orientation is correct for vision 
        processing and field-relative driving.
        """
        alliance = DriverStation.getAlliance()
        if alliance == DriverStation.Alliance.kBlue:
            self.pigeon.set_yaw(0, 0.1)
        elif alliance == DriverStation.Alliance.kRed:
            self.pigeon.set_yaw(180, 0.1)
        else:
            self.pigeon.set_yaw(0,0.1)
            self.logger.info("Warning: Alliance not found at start: Defaulting to Blue (0 degrees)")