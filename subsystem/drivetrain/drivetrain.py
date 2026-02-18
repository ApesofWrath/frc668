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
        # self.logger.info(f"DriverStation attached: {wpilib.DriverStation.isDSAttached()}")
        # alliance_color = wpilib.DriverStation.getAlliance()
        # if alliance_color is None:
        #     self.logger.error("Failed to get alliance from DriverStation")
        #     return
        # self.logger.info(f"Alliance color {alliance_color}")
        self.logger.info("Setting blue alliance perspective rotation")
        self.set_operator_perspective_forward(
            constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
            # if alliance_color == wpilib.DriverStation.Alliance.kRed
            # else constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
        )


    def reset_gyro_yaw(self) -> None:
        result = self.pigeon2.set_yaw(0)
        if not result.is_ok():
            self.logger.warning("Failed to reset gyro")

    def execute(self) -> None:
        pass

    def isManual(self):
        return True
