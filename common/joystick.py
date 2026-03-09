from __future__ import annotations

import math
from dataclasses import dataclass

import wpilib
import wpimath

import subsystem
from subsystem.shooter import constants

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

    def shouldResetOrientation(self) -> bool:
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


class OperatorController:
    def __init__(
        self,
        controller: wpilib.XboxController,
    ) -> None:
        self._controller = controller
        # Manual mode (turned on automatically when a preset is pressed).
        self._manual_enabled = False
        # Bumpers are only active for fine adjustments when this flag is True.
        # Controlled by the View/Back button.
        self._bumpers_enabled = False

    def update(self) -> None:
        # Toggle manual+bumper-enabled mode with the View/Back button. This
        # controls whether presets and bumpers are active.
        if self._controller.getBackButtonPressed():
            self._bumpers_enabled = not self._bumpers_enabled
            # Mirror manual mode to the bumpers flag: when bumpers are enabled
            # via the View button, manual mode should also be enabled (and
            # vice-versa when toggled off).
            self._manual_enabled = self._bumpers_enabled

    def is_manual(self) -> bool:
        return bool(self._manual_enabled)

    def enable_manual(self) -> None:
        self._manual_enabled = True

    def disable_manual(self) -> None:
        self._manual_enabled = False


    def get_target_angle(self, current_angle: float | None = None) -> float | None:
        if not self.is_manual():
            return None

        delta = self.get_fine_adjustment()
        if delta != 0.0:
            if current_angle is None:
                return None
            return max(-180.0, min(180.0, float(current_angle) + delta))

        return None
    
    def get_preset(self) -> dict | None:
        """Return preset dict when X/Y/A/B pressed while manual is enabled.

        Returns: {'turret_deg', 'hood_deg', 'flywheel_rpm'} or None.
        """
        # If any preset button is pressed, automatically enable manual mode
        # (this indicates operator wants manual/preset control and disables
        # auto-alignment). Return the preset even if manual wasn't previously
        # enabled.
        if self._controller.getXButton():
            # Pressing a preset implies operator wants manual control and
            self._manual_enabled = True
            return constants.OPERATOR_PRESET_X
        if self._controller.getYButton():
            self._manual_enabled = True
            return constants.OPERATOR_PRESET_Y
        if self._controller.getAButton():
            self._manual_enabled = True
            return constants.OPERATOR_PRESET_A
        if self._controller.getBButton():
            self._manual_enabled = True
            return constants.OPERATOR_PRESET_B

        return None
