import math
import wpimath
import wpilib
from wpimath.geometry import Rotation2d

from phoenix6 import hardware, swerve

from subsystem.drivetrain import constants


# Blue alliance sees forward as 0 degrees (toward red alliance wall)
_BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
"""Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
# Red alliance sees forward as 180 degrees (toward blue alliance wall)
_RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
"""Red alliance sees forward as 180 degrees (toward blue alliance wall)"""


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

        # Keep track if we've ever applied the operator perspective before or not
        self._has_applied_operator_perspective = False
        

    def execute(self) -> None:
        # Periodically try to apply the operator perspective
        # if we haven't yet or if we're currently disabled.
        if not self._has_applied_operator_perspective or wpilib.DriverStation.isDisabled():
            alliance_color = wpilib.DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                   _RED_ALLIANCE_PERSPECTIVE_ROTATION
                   if alliance_color == wpilib.DriverStation.Alliance.kRed
                   else _BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        

    def isManual(self):
        return True
