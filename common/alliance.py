import logging
import typing

import wpilib


class AllianceFetcher:
    """Fetches and caches the current driver station alliance."""

    def __init__(self) -> None:
        self._alliance: typing.Optional[wpilib.DriverStation.Alliance] = None

    def setup(self) -> None:
        """Try once during setup to resolve alliance."""
        self.maybeFetchAlliance()

    def execute(self) -> None:
        """Keep trying to resolve alliance until we know it."""
        self.maybeFetchAlliance()

    def maybeFetchAlliance(self) -> None:
        # If we have our alliance, we already fetched it.
        if self._alliance:
            return

        # If not, try to get the alliance.
        self._alliance = wpilib.DriverStation.getAlliance()

        # If we got it, log it once.
        if self._alliance:
            self.logger.info(f"We are alliance: {self._alliance}")

    def getAlliance(self) -> typing.Optional[wpilib.DriverStation.Alliance]:
        return self._alliance
