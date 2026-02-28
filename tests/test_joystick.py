import math
import pytest

from common import joystick


class TestDriveCommand:
    def test_default_values(self):
        """DriveCommand has zero default values."""
        command = joystick.DriveCommand()
        assert command.vx == 0.0
        assert command.vy == 0.0
        assert command.omega == 0.0

    def test_custom_values(self):
        """DriveCommand can be initialized with custom values."""
        command = joystick.DriveCommand(vx=1.0, vy=2.0, omega=3.0)
        assert command.vx == 1.0
        assert command.vy == 2.0
        assert command.omega == 3.0


class TestDriverControllerInit:
    def test_init_stores_controller_and_options(self, mocker):
        """DriverController stores the controller and options."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()

        driver = joystick.DriverController(mock_controller, mock_options)

        assert driver._controller is mock_controller
        assert driver._options is mock_options
        assert isinstance(driver._command, joystick.DriveCommand)


class TestShouldResetOrientation:
    def test_returns_true_when_start_button_released(self, mocker):
        """Returns True when start button is released."""
        mock_controller = mocker.Mock()
        mock_controller.getStartButtonReleased.return_value = True
        mock_options = mocker.Mock()

        driver = joystick.DriverController(mock_controller, mock_options)

        assert driver.should_reset_orientation() is True
        mock_controller.getStartButtonReleased.assert_called_once()

    def test_returns_false_when_start_button_not_released(self, mocker):
        """Returns False when start button is not released."""
        mock_controller = mocker.Mock()
        mock_controller.getStartButtonReleased.return_value = False
        mock_options = mocker.Mock()

        driver = joystick.DriverController(mock_controller, mock_options)

        assert driver.should_reset_orientation() is False


class TestFilterInput:
    def test_zero_input_returns_zero(self, mocker):
        """Zero input returns zero."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        result = driver._filter_input(0.0)
        assert result == 0.0

    def test_positive_input_is_squared(self, mocker):
        """Positive input is squared."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        # 0.5 squared = 0.25, which is above deadband (0.0225)
        result = driver._filter_input(0.5, apply_deadband=False)
        assert result == 0.25

    def test_negative_input_preserves_sign(self, mocker):
        """Negative input is squared but preserves sign."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        # -0.5 squared = -0.25 (sign preserved)
        result = driver._filter_input(-0.5, apply_deadband=False)
        assert result == -0.25

    def test_full_positive_input_returns_one(self, mocker):
        """Full input (1.0) returns 1.0."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        result = driver._filter_input(1.0, apply_deadband=False)
        assert result == 1.0

    def test_full_negative_input_returns_negative_one(self, mocker):
        """Full negative input (-1.0) returns -1.0."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        result = driver._filter_input(-1.0, apply_deadband=False)
        assert result == -1.0

    def test_small_input_filtered_by_deadband(self, mocker):
        """Small input within deadband returns zero."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        # 0.1 squared = 0.01, which is below deadband (0.15^2 = 0.0225)
        result = driver._filter_input(0.1, apply_deadband=True)
        assert result == 0.0

    def test_input_above_deadband_passes_through(self, mocker):
        """Input above deadband is filtered but not zeroed."""
        mock_controller = mocker.Mock()
        mock_options = mocker.Mock()
        driver = joystick.DriverController(mock_controller, mock_options)

        # 0.5 squared = 0.25, above deadband
        result = driver._filter_input(0.5, apply_deadband=True)
        assert result != 0.0
        assert result > 0.0


class TestGetDriveCommand:
    def test_zero_input_returns_zero_command(self, mocker):
        """Zero joystick input returns zero drive command."""
        mock_controller = mocker.Mock()
        mock_controller.getLeftY.return_value = 0.0
        mock_controller.getLeftX.return_value = 0.0
        mock_controller.getRightX.return_value = 0.0
        mock_controller.getLeftBumper.return_value = False

        mock_options = mocker.Mock()
        mock_options.max_linear_speed_meters_per_second = 6.0
        mock_options.max_angular_speed_radians_per_second = 6.0

        driver = joystick.DriverController(mock_controller, mock_options)
        command = driver.get_drive_command()

        assert command.vx == 0.0
        assert command.vy == 0.0
        assert command.omega == 0.0

    def test_full_forward_input_returns_max_vx(self, mocker):
        """Full forward input returns maximum vx."""
        mock_controller = mocker.Mock()
        mock_controller.getLeftY.return_value = -1.0  # Forward is negative Y
        mock_controller.getLeftX.return_value = 0.0
        mock_controller.getRightX.return_value = 0.0
        mock_controller.getLeftBumper.return_value = False

        mock_options = mocker.Mock()
        mock_options.max_linear_speed_meters_per_second = 6.0
        mock_options.max_angular_speed_radians_per_second = 6.0

        driver = joystick.DriverController(mock_controller, mock_options)
        command = driver.get_drive_command()

        assert command.vx == pytest.approx(6.0, rel=0.01)
        assert command.vy == 0.0
        assert command.omega == 0.0

    def test_slow_mode_reduces_speed(self, mocker):
        """Left bumper activates slow mode (0.1 modifier)."""
        mock_controller = mocker.Mock()
        mock_controller.getLeftY.return_value = -1.0
        mock_controller.getLeftX.return_value = 0.0
        mock_controller.getRightX.return_value = 0.0
        mock_controller.getLeftBumper.return_value = True

        mock_options = mocker.Mock()
        mock_options.max_linear_speed_meters_per_second = 6.0
        mock_options.max_angular_speed_radians_per_second = 6.0

        driver = joystick.DriverController(mock_controller, mock_options)
        command = driver.get_drive_command()

        # Slow mode = 0.1 modifier, so max vx should be 0.6
        assert command.vx == pytest.approx(0.6, rel=0.01)

    def test_full_strafe_input_returns_max_vy(self, mocker):
        """Full strafe input returns maximum vy."""
        mock_controller = mocker.Mock()
        mock_controller.getLeftY.return_value = 0.0
        mock_controller.getLeftX.return_value = -1.0  # Left is negative X
        mock_controller.getRightX.return_value = 0.0
        mock_controller.getLeftBumper.return_value = False

        mock_options = mocker.Mock()
        mock_options.max_linear_speed_meters_per_second = 6.0
        mock_options.max_angular_speed_radians_per_second = 6.0

        driver = joystick.DriverController(mock_controller, mock_options)
        command = driver.get_drive_command()

        assert command.vx == 0.0
        assert command.vy == pytest.approx(6.0, rel=0.01)
        assert command.omega == 0.0

    def test_full_rotation_input_returns_max_omega(self, mocker):
        """Full rotation input returns maximum omega."""
        mock_controller = mocker.Mock()
        mock_controller.getLeftY.return_value = 0.0
        mock_controller.getLeftX.return_value = 0.0
        mock_controller.getRightX.return_value = -1.0  # CCW rotation
        mock_controller.getLeftBumper.return_value = False

        mock_options = mocker.Mock()
        mock_options.max_linear_speed_meters_per_second = 6.0
        mock_options.max_angular_speed_radians_per_second = 6.0

        driver = joystick.DriverController(mock_controller, mock_options)
        command = driver.get_drive_command()

        assert command.vx == 0.0
        assert command.vy == 0.0
        assert command.omega == pytest.approx(6.0, rel=0.01)

    def test_reuses_command_object(self, mocker):
        """get_drive_command reuses the same DriveCommand object."""
        mock_controller = mocker.Mock()
        mock_controller.getLeftY.return_value = 0.0
        mock_controller.getLeftX.return_value = 0.0
        mock_controller.getRightX.return_value = 0.0
        mock_controller.getLeftBumper.return_value = False

        mock_options = mocker.Mock()
        mock_options.max_linear_speed_meters_per_second = 6.0
        mock_options.max_angular_speed_radians_per_second = 6.0

        driver = joystick.DriverController(mock_controller, mock_options)
        command1 = driver.get_drive_command()
        command2 = driver.get_drive_command()

        assert command1 is command2
