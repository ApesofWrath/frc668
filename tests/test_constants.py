import dataclasses
import pytest

import constants


def test_get_robot_constants_alphabot(mocker):
    """All constants load on alphabot."""
    # When `robotpy` runs our tests, isSimulation returns True. Force it to
    # False so we can test the non-sim path.
    mocker.patch("wpilib.RobotBase.isSimulation", return_value=False)

    mock_run = mocker.patch("subprocess.run")
    mock_run.return_value = mocker.Mock(stdout="023AC96C\n", returncode=0)

    robot_constants: constants.RobotConstants = constants.get_robot_constants()

    assert robot_constants.serial == "023AC96C"
    assert robot_constants.drivetrain is not None
    assert robot_constants.intake is not None
    assert robot_constants.shooter is not None


def test_get_robot_constants_test_chassis(mocker):
    """Only drivetrain constants load on test chassis."""
    # When `robotpy` runs our tests, isSimulation returns True. Force it to
    # False so we can test the non-sim path.
    mocker.patch("wpilib.RobotBase.isSimulation", return_value=False)

    mock_run = mocker.patch("subprocess.run")
    mock_run.return_value = mocker.Mock(stdout="0323800E\n", returncode=0)

    robot_constants: constants.RobotConstants = constants.get_robot_constants()

    assert robot_constants.serial == "0323800E"
    assert robot_constants.drivetrain is not None
    assert robot_constants.intake is None
    assert robot_constants.shooter is None


def test_get_robot_constants_sim_uses_default(mocker):
    """In simulation, we use the default serial."""
    # Force simulation.
    mocker.patch("wpilib.RobotBase.isSimulation", return_value=True)

    mock_warn = mocker.patch("wpilib.reportWarning")

    robot_constants: constants.RobotConstants = constants.get_robot_constants()

    mock_warn.assert_called_once_with(
        "Running in simulation - using default robot constants", False
    )
    assert robot_constants.serial == constants.DEFAULT_ROBOT_SERIAL


def test_constants_are_immutable(mocker):
    """Constants cannot be modified."""
    # When `robotpy` runs our tests, isSimulation returns True. Force it to
    # False so we can test the non-sim path.
    mocker.patch("wpilib.RobotBase.isSimulation", return_value=False)

    mock_run = mocker.patch("subprocess.run")
    mock_run.return_value = mocker.Mock(stdout="023AC96C\n", returncode=0)

    robot_constants: constants.RobotConstants = constants.get_robot_constants()

    with pytest.raises(dataclasses.FrozenInstanceError):
        robot_constants.shooter.hopper.left_k_p = 42
