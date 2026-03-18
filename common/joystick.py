from __future__ import annotations

import math
from dataclasses import dataclass

import wpilib
import wpimath


@dataclass
class DriveCommand:
    vx: float = 0.0  # meters per second
    vy: float = 0.0  # meters per second
    omega: float = 0.0  # radians per second


class DriverController:
    """This class handles driver input for the robot.

    All controller mappings for the driver controller are defined here.
    """

    DEADBAND: float = math.pow(0.15, 2)
    FAST: float = 1.0
    SLOW: float = 0.1

    def __init__(
        self,
        controller: wpilib.XboxController,
        options: "subsystem.drivetrain.constants.DriveOptions",
    ) -> None:
        self._controller: wpilib.XboxController = controller
        self._options: "subsystem.drivetrain.constants.DriveOptions" = options
        self._command: DriveCommand = DriveCommand()

    def resetOrientation(self) -> bool:
        """Returns True if the robot's orientation should be reset.

        When the driver releases the start button, it is an indication that the
        robot's orientation must be reset, typically with the seed_field_centric
        method on the drivetrain class.
        """
        return self._controller.getStartButtonReleased()

    def toggleIntake(self) -> bool:
        """Returns True if the intake should be run.

        When the driver holds down the right bumper, we want to run the intake.
        """
        return self._controller.getRightBumperPressed()

    def feedFuel(self) -> bool:
        """Returns True if fuel should be fed to the shooter.

        When the driver holds down the right trigger, we want to shoot.
        """
        return self._controller.getRightTriggerAxis()

    def getDriveCommand(self) -> DriveCommand:
        """Returns the drivetrain commands corresponding to current user input.

        The raw inputs from the joystick are filtered and normalized to the
        range of speeds specified in the options.
        """
        modifier = self.SLOW if self._controller.getLeftBumper() else self.FAST
        self._command.vx = (
            -self._filterInput(self._controller.getLeftY())
            * self._options.max_linear_speed_meters_per_second
            * modifier
        )
        self._command.vy = (
            -self._filterInput(self._controller.getLeftX())
            * self._options.max_linear_speed_meters_per_second
            * modifier
        )
        self._command.omega = (
            -self._filterInput(self._controller.getRightX())
            * self._options.max_angular_speed_radians_per_second
            * modifier
        )
        return self._command

    def shootFromLeftTrench(self) -> bool:
        """Indicates if the driver wants to shoot from the left trench.

        If this returns True, the robot should command its mechanisms to the
        necessary presets to be able to shoot from the left trench position.
        """
        return self._controller.getXButton()

    def shootFromRightTrench(self) -> bool:
        """Indicates if the driver wants to shoot from the right trench.

        If this returns True, the robot should command its mechanisms to the
        necessary presets to be able to shoot from the right trench position.
        """
        return self._controller.getBButton()

    def shootFromBehindTower(self) -> bool:
        """Indicates if the driver wants to shoot from behind the tower.

        If this returns True, the robot should command its mechanisms to the
        necessary presets to be able to shoot from behind the tower.
        """
        return self._controller.getAButton()

    def brake(self) -> bool:
        """Returns True if the driver wants the robot to brake.

        This can be useful when shooting from a stationary spot, so we don't get
        pushed around too easily.
        """
        return self._controller.getYButton()

    def _filterInput(self, input: float, apply_deadband: bool = True) -> float:
        """Filter the joystick input with a squared scaling and deadband.

        This function squares the input while preserving its sign to provide
        finer control at lower values. If `apply_deadband` is True, it applies a
        deadband to ignore small inputs that may result from controller drift.

        Args:
            controller_input:
                The raw input from the controller, ranging from -1.0 to 1.0.
            apply_deadband:
                Whether to apply a deadband to the input. Defaults to True.

        Returns:
            A float that is the filtered controller input.
        """
        squared_input = math.copysign(math.pow(input, 2), input)
        if apply_deadband:
            return wpimath.applyDeadband(squared_input, self.DEADBAND)
        else:
            return squared_input
