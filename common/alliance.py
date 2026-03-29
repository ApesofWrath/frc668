import typing

import magicbot
import wpilib

from common import datalog


class AllianceFetcher:
    """Fetches the current alliance and provides some helpers."""

    def isBlueAlliance(self) -> bool:
        """Returns True if we are the blue alliance."""
        return self.get_alliance() == wpilib.DriverStation.Alliance.kBlue

    def isRedAlliance(self) -> bool:
        """Returns True if we are the red alliance."""
        return self.get_alliance() == wpilib.DriverStation.Alliance.kRed

    @magicbot.feedback
    def get_alliance(self) -> wpilib.DriverStation.Alliance:
        return wpilib.DriverStation.getAlliance()
