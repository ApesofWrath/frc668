import math

import wpilib
import wpimath
from phoenix6 import hardware, swerve

from subsystem.drivetrain import constants


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

    def setup(self) -> None:
        """Apply the operator perspective based on alliance color."""
        alliance_color = wpilib.DriverStation.getAlliance()
        if alliance_color is None:
            self.logger.error("Failed to get alliance from DriverStation")
            return
        self.logger.info(f"Alliance color {alliance_color}")
        self.set_operator_perspective_forward(
            constants.RED_ALLIANCE_PERSPECTIVE_ROTATION
            if alliance_color == wpilib.DriverStation.Alliance.kRed
            else constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
        )

    def execute(self) -> None:
        pass

    def isManual(self):
        return True
