import typing

import wpilib


class AllianceFetcher:
    """Fetches the current alliance and provides some helpers."""

    def __init__(self) -> None:
        self._alliance: typing.Optional[wpilib.DriverStation.Alliance] = None
        self._warning_timer = wpilib.Timer()
        self._warning_timer.start()

    def setup(self) -> None:
        """Try once during setup to resolve alliance."""
        self._alliance = wpilib.DriverStation.getAlliance()

    def execute(self) -> None:
        """Keep querying our alliance.

        We may not have gotten it at startup, or it may have changed.
        """
        self._alliance = wpilib.DriverStation.getAlliance()

        if not self.hasAlliance():
            if self._warning_timer.advanceIfElapsed(1.0):
                self.logger.warning(
                    "Haven't received alliance information from DriverStation"
                )

    def isBlueAlliance(self) -> bool:
        """Returns True if we are the blue alliance."""
        return self._alliance == wpilib.DriverStation.Alliance.kBlue

    def isRedAlliance(self) -> bool:
        """Returns True if we are the red alliance."""
        return self._alliance == wpilib.DriverStation.Alliance.kRed

    def hasAlliance(self) -> bool:
        """Returns True if we have received alliance info from the DS."""
        return self._alliance is not None
