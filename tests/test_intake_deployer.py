import logging
from unittest import mock

import phoenix6
import pytest
from magicbot import magic_tunable

from subsystem import intake

_GET_TIME = "magicbot.state_machine.getTime"


class MockDeployConstants:
    """Minimal stand-in for robot_constants.intake used by IntakeDeployer.setup."""

    def __init__(self):
        self.deploy_encoder_direction = (
            phoenix6.signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.deploy_encoder_can_id = 61
        self.deploy_sensor_to_mechanism_ratio = 3.333
        self.deploy_rotor_to_sensor_ratio = 25.0
        self.deploy_motor_inverted = (
            phoenix6.signals.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.deploy_motor_supply_current_limit = 10.0


@pytest.fixture
def mock_constants():
    robot_constants = mock.MagicMock()
    robot_constants.intake = MockDeployConstants()
    return robot_constants


@pytest.fixture
def mock_deploy_motor():
    motor = mock.MagicMock(spec=phoenix6.hardware.TalonFX)
    motor.configurator = mock.MagicMock()
    return motor


@pytest.fixture
def mock_deploy_encoder():
    encoder = mock.MagicMock(spec=phoenix6.hardware.CANcoder)
    encoder.configurator = mock.MagicMock()
    pos_signal = mock.MagicMock()
    pos_signal.value = 0.0
    encoder.get_position.return_value = pos_signal
    return encoder


@pytest.fixture
def mock_intake():
    return mock.MagicMock()


@pytest.fixture
def deployer(
    mock_constants, mock_deploy_motor, mock_deploy_encoder, mock_intake
):
    """Fresh IntakeDeployer with mocks injected and setup() called."""
    d = intake.IntakeDeployer()
    d.robot_constants = mock_constants
    d.intake_deploy_motor = mock_deploy_motor
    d.intake_deploy_encoder = mock_deploy_encoder
    d.intake = mock_intake
    d.logger = logging.getLogger("IntakeDeployer")
    magic_tunable.setup_tunables(d, "intake_deployer")
    d.setup()
    return d


class TestIntakeDeployerSetup:
    def test_encoder_configured(self, deployer, mock_deploy_encoder):
        """setup() applies a CANcoderConfiguration to the deploy encoder."""
        mock_deploy_encoder.configurator.apply.assert_called_once()
        config = mock_deploy_encoder.configurator.apply.call_args[0][0]
        assert isinstance(config, phoenix6.configs.CANcoderConfiguration)

    def test_motor_configured(self, deployer, mock_deploy_motor):
        """setup() applies a TalonFXConfiguration to the deploy motor."""
        mock_deploy_motor.configurator.apply.assert_called_once()
        config = mock_deploy_motor.configurator.apply.call_args[0][0]
        assert isinstance(config, phoenix6.configs.TalonFXConfiguration)

    def test_encoder_position_zeroed(self, deployer, mock_deploy_encoder):
        """setup() zeroes the encoder so the mechanism starts at position 0."""
        mock_deploy_encoder.set_position.assert_called_once_with(0.0)

    def test_initial_deployed_flag(self, deployer):
        """_deployed starts False before the state machine has run."""
        assert deployer._deployed is False


class TestDeploy:
    def test_deploy_engages_when_not_deployed(self, deployer):
        """deploy() starts the state machine when the intake hasn't deployed yet."""
        deployer.deploy()
        # engage() sets current_state but __engaged is only set by execute()
        assert deployer.current_state == "deploying"

    def test_deploy_does_not_engage_when_already_deployed(self, deployer):
        """deploy() is a no-op once the intake is already deployed."""
        deployer._deployed = True
        deployer.deploy()
        assert deployer.current_state == ""


class TestDeployingState:
    def _step(self, deployer, fake_time):
        """Execute one cycle of the state machine at the given FPGA time."""
        with mock.patch(_GET_TIME, return_value=fake_time):
            deployer.engage()
            deployer.execute()

    def test_motor_and_intake_active_while_deploying(
        self, deployer, mock_deploy_motor, mock_deploy_encoder, mock_intake
    ):
        """While deploying, the motor runs at 0.25 and the intake rollers are active."""
        mock_deploy_encoder.get_position.return_value.value = 0.0
        self._step(deployer, 0.0)

        mock_deploy_motor.set.assert_called_with(0.25)
        mock_intake.setActive.assert_called_with(True)

    def test_transitions_to_deployed_at_threshold(
        self, deployer, mock_deploy_encoder
    ):
        """Encoder >= 0.85 triggers transition to the deployed state."""
        mock_deploy_encoder.get_position.return_value.value = 0.85
        self._step(deployer, 0.0)

        assert deployer.current_state == "deployed"

    def test_stays_deploying_below_threshold(
        self, deployer, mock_deploy_encoder
    ):
        """Encoder below 0.85 keeps the machine in the deploying state."""
        mock_deploy_encoder.get_position.return_value.value = 0.84
        self._step(deployer, 0.0)

        assert deployer.current_state == "deploying"

    def test_transitions_to_timed_out(self, deployer, mock_deploy_encoder):
        """If the encoder never reaches 0.85 within 10 s, transition to timed_out."""
        mock_deploy_encoder.get_position.return_value.value = 0.0
        self._step(deployer, 0.0)
        self._step(deployer, 10.0)

        assert deployer.current_state == "timed_out"


class TestDeployedState:
    def _run_to_deployed(self, deployer, mock_deploy_encoder):
        mock_deploy_encoder.get_position.return_value.value = 0.85
        with mock.patch(_GET_TIME, return_value=0.0):
            deployer.engage()
            deployer.execute()
        # Now in "deployed" state; run it
        with mock.patch(_GET_TIME, return_value=0.02):
            deployer.engage()
            deployer.execute()

    def test_sets_deployed_flag(self, deployer, mock_deploy_encoder):
        """deployed state sets the _deployed flag to True."""
        self._run_to_deployed(deployer, mock_deploy_encoder)
        assert deployer._deployed is True

    def test_stops_motor(
        self, deployer, mock_deploy_motor, mock_deploy_encoder
    ):
        """deployed state commands the motor to stop."""
        self._run_to_deployed(deployer, mock_deploy_encoder)
        mock_deploy_motor.set.assert_called_with(0.0)

    def test_state_machine_finishes(self, deployer, mock_deploy_encoder):
        """deployed state calls done(), ending the state machine."""
        self._run_to_deployed(deployer, mock_deploy_encoder)
        assert not deployer.is_executing


class TestTimedOutState:
    def _run_to_timed_out(self, deployer, mock_deploy_encoder):
        mock_deploy_encoder.get_position.return_value.value = 0.0
        with mock.patch(_GET_TIME, return_value=0.0):
            deployer.engage()
            deployer.execute()
        with mock.patch(_GET_TIME, return_value=10.0):
            deployer.engage()
            deployer.execute()
        # Now in "timed_out"; execute it
        with mock.patch(_GET_TIME, return_value=10.02):
            deployer.engage()
            deployer.execute()

    def test_does_not_set_deployed_flag(self, deployer, mock_deploy_encoder):
        """timed_out does NOT mark the intake as deployed."""
        self._run_to_timed_out(deployer, mock_deploy_encoder)
        assert deployer._deployed is False

    def test_stops_motor(
        self, deployer, mock_deploy_motor, mock_deploy_encoder
    ):
        """timed_out commands the motor to stop."""
        self._run_to_timed_out(deployer, mock_deploy_encoder)
        mock_deploy_motor.set.assert_called_with(0.0)

    def test_state_machine_finishes(self, deployer, mock_deploy_encoder):
        """timed_out calls done(), ending the state machine."""
        self._run_to_timed_out(deployer, mock_deploy_encoder)
        assert not deployer.is_executing


class TestGetEncoderRotation:
    def test_returns_encoder_position(self, deployer, mock_deploy_encoder):
        """Feedback method returns the raw encoder position value."""
        mock_deploy_encoder.get_position.return_value.value = 0.42
        assert deployer.get_encoder_position_rotations() == pytest.approx(0.42)
