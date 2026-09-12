import wpilib


class AllianceFetcher:
    """Fetches the current alliance and provides some helpers."""

    def is_blue_alliance(self) -> bool:
        """Returns True if we are the blue alliance."""
        return self.get_alliance() == wpilib.DriverStation.Alliance.kBlue

    def is_red_alliance(self) -> bool:
        """Returns True if we are the red alliance."""
        return self.get_alliance() == wpilib.DriverStation.Alliance.kRed

    def get_alliance(self) -> wpilib.DriverStation.Alliance:
        """Returns our alliance as reported by the DriverStation."""
        return wpilib.DriverStation.getAlliance()
