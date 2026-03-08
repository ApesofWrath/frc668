import pytest
from unittest import mock

import phoenix6

from subsystem.intake import Intake


class MockIntakeConstants:
    """Concrete object for robot_constants.intake so we can assert exact config values."""

    def __init__(self):
        self.active_roller_speed_rps = 80.0
        self.roller_motor_inverted = (
            phoenix6.signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.k_s = 0.25
        self.k_v = 0.12
        self.k_a = 0.0
        self.k_p = 0.2
        self.k_i = 0.0
        self.k_d = 0.0


@pytest.fixture
def mock_constants():
    """Mock for constants.RobotConstants."""
    robot_constants = mock.MagicMock()
    robot_constants.intake = MockIntakeConstants()
    return robot_constants


@pytest.fixture
def mock_motor():
    """Mock for phoenix6.hardware.TalonFX."""
    motor = mock.MagicMock(spec=phoenix6.hardware.TalonFX)
    motor.configurator = mock.MagicMock()

    # Simulate the velocity status signal (used by get_measured_speed)
    vel_signal = mock.MagicMock()
    vel_signal.value = 42.0
    motor.get_velocity.return_value = vel_signal
    return motor


@pytest.fixture
def intake(mock_constants, mock_motor):
    """Fresh Intake instance with mocks injected and setup() already called."""
    component = Intake()
    component.robot_constants = mock_constants
    component.intake_roller_motor = mock_motor
    component.setup()
    return component


class TestIntake:
    """Unit tests for the Intake magicbot component."""

    def test_setup_applies_configuration(
        self, intake, mock_constants, mock_motor
    ):
        """Verify TalonFXConfigurator.apply() is called with the correct chained config."""
        mock_motor.configurator.apply.assert_called_once()

        config = mock_motor.configurator.apply.call_args[0][0]
        assert isinstance(config, phoenix6.configs.TalonFXConfiguration)

        # Motor output inversion
        assert (
            config.motor_output.inverted == mock_constants.intake.roller_motor_inverted
        )

        # Slot 0 PID/FF gains
        s0 = config.slot0
        assert s0.k_s == mock_constants.intake.k_s
        assert s0.k_v == mock_constants.intake.k_v
        assert s0.k_a == mock_constants.intake.k_a
        assert s0.k_p == mock_constants.intake.k_p
        assert s0.k_i == mock_constants.intake.k_i
        assert s0.k_d == mock_constants.intake.k_d

    def test_setup_creates_control_request(self, intake):
        """The VelocityVoltage request object should exist after setup."""
        assert hasattr(intake, "_request")
        assert isinstance(intake._request, phoenix6.controls.VelocityVoltage)

    def test_initial_state(self, intake):
        """After setup, the intake should be inactive with the default speed from constants."""
        assert intake._active is False
        assert intake._active_roller_speed_rps == 80.0

    def test_set_active_and_toggle(self, intake):
        """State control methods should work as expected."""
        intake.setActive(True)
        assert intake._active is True

        intake.toggleActive()
        assert intake._active is False

        intake.toggleActive()
        assert intake._active is True

        intake.toggleActive()
        assert intake._active is False

        intake.setActive(False)
        assert intake._active is False

    def test_set_speed(self, intake):
        """setSpeed should update the target roller speed (including None default)."""
        intake.setSpeed(55.5)
        assert intake._active_roller_speed_rps == 55.5

        # As-written behavior (no guard against None)
        intake.setSpeed()
        assert intake._active_roller_speed_rps is None

    def test_on_enable_on_disable_reset_active(self, intake):
        """Lifecycle hooks must force the intake to a safe (stopped) state."""
        intake.setActive(True)

        intake.on_enable()
        assert intake._active is False

        intake.setActive(True)
        intake.on_disable()
        assert intake._active is False

    def test_execute_inactive_sets_zero_velocity(self, intake, mock_motor):
        """When inactive, execute() must command 0 RPS."""
        intake.setActive(False)
        intake.execute()

        mock_motor.set_control.assert_called_once()
        request = mock_motor.set_control.call_args[0][0]
        assert isinstance(request, phoenix6.controls.VelocityVoltage)
        assert request.velocity == 0.0

    def test_execute_active_sets_requested_velocity(self, intake, mock_motor):
        """When active, execute() must command the current _active_roller_speed_rps."""
        intake.setActive(True)
        intake.setSpeed(67.0)
        intake.execute()

        mock_motor.set_control.assert_called_once()
        request = mock_motor.set_control.call_args[0][0]
        assert request.velocity == pytest.approx(67.0)

    def test_get_measured_speed(self, intake, mock_motor):
        """Feedback method should return the raw velocity or 0.0 when falsy."""
        # Normal case
        mock_motor.get_velocity.return_value.value = 123.4
        assert intake.get_measured_speed() == 123.4

        # Falsy value handling (per original implementation)
        mock_motor.get_velocity.return_value.value = None
        assert intake.get_measured_speed() == 0.0

        mock_motor.get_velocity.return_value.value = 0
        assert intake.get_measured_speed() == 0.0
