import wpilib

from common import datalog


class AllianceFetcher:
    """Fetches the current alliance and provides some helpers."""

    def isBlueAlliance(self) -> bool:
        """Returns True if we are the blue alliance."""
        return self.getAlliance() == wpilib.DriverStation.Alliance.kBlue

    def isRedAlliance(self) -> bool:
        """Returns True if we are the red alliance."""
        return self.getAlliance() == wpilib.DriverStation.Alliance.kRed

    def getAlliance(self) -> wpilib.DriverStation.Alliance:
        """Returns our alliance as reported by the DriverStation."""
        return wpilib.DriverStation.getAlliance()
