import subprocess
from dataclasses import dataclass

import wpilib

# TODO: Set this to the competition robot's serial number.
DEFAULT_ROBOT_SERIAL = "023AC96C"


@dataclass(frozen=True)
class RobotConstants:
    serial: str = "Unknown"
    drivetrain: "subsystem.drivetrain.constants.DrivetrainConstants | None" = (
        None
    )
    intake: "subsystem.intake.constants.IntakeConstants | None" = None
    shooter: "subsystem.shooter.constants.ShooterConstants | None" = None


def get_robot_constants() -> RobotConstants:
    """Fetches robot constants based on serial number.

    Attempts to read the serial number, and fetches the matching constants from
    various subsystems and returns them.

    In simulation mode, instead of attempting to read the serial, it uses the
    default.

    Returns:
        A RobotConstants object containing all the constants found for the
        determined serial number.
    """
    if wpilib.RobotBase.isSimulation():
        robot_serial = DEFAULT_ROBOT_SERIAL
        wpilib.reportWarning(
            "Running in simulation - using default robot constants", False
        )
    else:
        try:
            result = subprocess.run(
                ["/sbin/fw_printenv", "-n", "serial#"],
                capture_output=True,
                text=True,
                timeout=1,
                check=True,
            )
            robot_serial = result.stdout.rstrip()
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as e:
            wpilib.reportError(
                f"Failed to get robot serial numbers: {e}", False
            )
            wpilib.reportWarning(
                f"Using constants for default serial number: {DEFAULT_ROBOT_SERIAL}",
                False,
            )
            robot_serial = DEFAULT_ROBOT_SERIAL

    # If these imports are moved to global scope, subsytem/shooter/__init__.py
    # gets executed, which imports Tuner classes, which triggers @tunable +
    # get_type_hints() while constants.py is still being loaded, and fails
    # because it can't find the RobotConstants type.
    #
    # Hence we import them lazily inside this function.
    from subsystem import drivetrain, intake, shooter

    drivetrain_constants: typing.Optional[drivetrain.DrivetrainConstants] = (
        drivetrain.DRIVETRAIN_CONSTANTS.get(robot_serial, None)
    )
    intake_constants: typing.Optional[intake.IntakeConstants] = (
        intake.INTAKE_CONSTANTS.get(robot_serial, None)
    )
    shooter_constants: typing.Optional[shooter.ShooterConstants] = (
        shooter.SHOOTER_CONSTANTS.get(robot_serial, None)
    )

    return RobotConstants(
        serial=robot_serial,
        drivetrain=drivetrain_constants,
        intake=intake_constants,
        shooter=shooter_constants,
    )
