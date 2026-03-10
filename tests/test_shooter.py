import pytest
from unittest import mock

from phoenix6 import swerve
from magicbot import magic_tunable

from subsystem import shooter


class MockFlywheelConstants:
    """Concrete object for robot_constants.shooter.flywheel."""

    def __init__(self):
        self.default_speed_rps = 20.0


class MockShooterConstants:
    """Concrete object for robot_constants.shooter."""

    def __init__(self):
        self.flywheel = MockFlywheelConstants()


@pytest.fixture
def mock_constants():
    """Mock for constants.RobotConstants."""
    robot_constants = mock.MagicMock()
    robot_constants.shooter = MockShooterConstants()
    return robot_constants


@pytest.fixture
def mock_turret():
    """Mock for shooter.Turret."""
    turret = mock.MagicMock()
    turret.get_measured_angle_degrees.return_value = 0.0
    return turret


@pytest.fixture
def mock_hood():
    """Mock for shooter.Hood."""
    hood = mock.MagicMock()
    hood.get_measured_angle_degrees.return_value = 0.0
    return hood


@pytest.fixture
def mock_flywheel():
    """Mock for shooter.Flywheel."""
    flywheel = mock.MagicMock()
    flywheel.get_measured_speed_rps.return_value = 20.0
    # Mock the flywheel_encoder.get_velocity().value
    flywheel.flywheel_encoder = mock.MagicMock()
    vel_signal = mock.MagicMock()
    vel_signal.value = 20.0
    flywheel.flywheel_encoder.get_velocity.return_value = vel_signal
    return flywheel


@pytest.fixture
def mock_hopper():
    """Mock for shooter.Hopper."""
    return mock.MagicMock()


@pytest.fixture
def mock_indexer():
    """Mock for shooter.Indexer."""
    return mock.MagicMock()


@pytest.fixture
def mock_drivetrain():
    """Mock for drivetrain.Drivetrain."""
    drivetrain = mock.MagicMock()
    # Mock chassis speeds for stationary robot
    chassis_speeds = mock.MagicMock(spec=swerve.ChassisSpeeds)
    chassis_speeds.vx = 0.0
    chassis_speeds.vy = 0.0
    drivetrain.get_robot_speed.return_value = chassis_speeds
    return drivetrain


@pytest.fixture
def mock_hub_tracker():
    """Mock for shooter.HubTracker."""
    hub_tracker = mock.MagicMock()
    hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
    hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
    hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
    return hub_tracker


@pytest.fixture
def shooter_sm(
    mock_constants,
    mock_turret,
    mock_hood,
    mock_flywheel,
    mock_hopper,
    mock_indexer,
    mock_drivetrain,
    mock_hub_tracker,
):
    """Fresh Shooter instance with mocks injected and setup() already called."""
    sm = shooter.Shooter()
    sm.robot_constants = mock_constants
    sm.turret = mock_turret
    sm.hood = mock_hood
    sm.flywheel = mock_flywheel
    sm.hopper = mock_hopper
    sm.indexer = mock_indexer
    sm.drivetrain = mock_drivetrain
    sm.hub_tracker = mock_hub_tracker
    # Provide a logger mock to avoid AttributeError
    sm.logger = mock.MagicMock()
    # Initialize magicbot tunables for state machine
    magic_tunable.setup_tunables(sm, "shooter")
    sm.setup()
    return sm


class TestShooter:
    """Unit tests for the Shooter magicbot StateMachine."""

    # -------------------------------------------------------------------------
    # Initial state tests
    # -------------------------------------------------------------------------

    def test_initial_state_is_idling(self, shooter_sm):
        """After engage+execute, the shooter state machine should be in 'idling'."""
        # State machine needs engage() + execute() to enter first state
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "idling"

    def test_setup_initializes_driver_wants_feed_false(self, shooter_sm):
        """After setup, _driver_wants_feed should be False."""
        assert shooter_sm._driver_wants_feed is False

    # -------------------------------------------------------------------------
    # setDriverWantsFeed tests
    # -------------------------------------------------------------------------

    def test_set_driver_wants_feed_true(self, shooter_sm):
        """setDriverWantsFeed(True) should set the flag to True."""
        shooter_sm.setDriverWantsFeed(True)
        assert shooter_sm._driver_wants_feed is True

    def test_set_driver_wants_feed_false(self, shooter_sm):
        """setDriverWantsFeed(False) should set the flag to False."""
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.setDriverWantsFeed(False)
        assert shooter_sm._driver_wants_feed is False

    # -------------------------------------------------------------------------
    # State transition tests: idling -> targeting
    # -------------------------------------------------------------------------

    def test_idling_to_targeting_when_driver_wants_feed(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_flywheel,
        mock_hopper,
        mock_indexer,
    ):
        """When driver wants feed, should transition from idling to targeting."""
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "targeting"
        # Verify idling behavior was executed before transition
        mock_hub_tracker.trackPosition.assert_called()
        mock_hub_tracker.trackSpeed.assert_called()

    def test_idling_stays_idling_when_driver_does_not_want_feed(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_flywheel,
        mock_hopper,
        mock_indexer,
    ):
        """When driver doesn't want feed, should remain in idling state."""
        shooter_sm.setDriverWantsFeed(False)
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "idling"
        mock_hub_tracker.trackPosition.assert_called_with(True)
        mock_hub_tracker.trackSpeed.assert_called_with(False)
        mock_hopper.setEnabled.assert_called_with(False)
        mock_indexer.setEnabled.assert_called_with(False)

    def test_idling_sets_flywheel_to_default_speed(
        self, shooter_sm, mock_flywheel, mock_constants
    ):
        """Idling state should set flywheel to default idle speed."""
        shooter_sm.engage()
        shooter_sm.execute()

        mock_flywheel.setTargetRps.assert_called_with(
            mock_constants.shooter.flywheel.default_speed_rps
        )

    # -------------------------------------------------------------------------
    # State transition tests: targeting -> shooting
    # -------------------------------------------------------------------------

    def test_targeting_to_shooting_when_ready_and_stationary(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When shooter is ready and robot is stationary, should transition to shooting."""
        # Set up conditions for shooter to be ready
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = (
            0.0  # Within tolerance
        )
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = (
            0.0  # Within tolerance
        )
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = (
            20.0  # Within tolerance
        )

        # Robot is stationary
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # First execute: idling -> targeting (scheduled)
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting checks ready & stationary, uses next_state_now -> shooting
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

    def test_targeting_stays_targeting_when_not_ready(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When shooter is not ready, should stay in targeting state."""
        # Set up conditions where turret is not at target
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 45.0
        mock_turret.get_measured_angle_degrees.return_value = (
            0.0  # Far from target
        )

        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "targeting"

    def test_targeting_stays_targeting_when_robot_moving(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When robot is moving, should stay in targeting state even if ready."""
        # Shooter is ready
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0

        # Robot is moving
        mock_drivetrain.get_robot_speed.return_value.vx = 1.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "targeting"

    # -------------------------------------------------------------------------
    # State transition tests: targeting -> idling
    # -------------------------------------------------------------------------

    def test_targeting_to_idling_when_driver_stops_wanting_feed(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When driver stops wanting feed in targeting, should go back to idling."""
        # Set up shooter not ready so it stays in targeting
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 45.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0

        # Go to targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Driver releases button - need engage() each loop
        shooter_sm.setDriverWantsFeed(False)
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "idling"

    # -------------------------------------------------------------------------
    # State transition tests: shooting -> idling
    # -------------------------------------------------------------------------

    def test_shooting_to_idling_when_driver_stops_wanting_feed(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When driver stops wanting feed in shooting, should go to idling."""
        # Set up to get to shooting state
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # First execute: idling -> targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting -> shooting (via next_state_now)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

        # Driver releases button - need engage() each loop
        shooter_sm.setDriverWantsFeed(False)
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "idling"

    # -------------------------------------------------------------------------
    # State transition tests: shooting -> targeting
    # -------------------------------------------------------------------------

    def test_shooting_to_targeting_when_shooter_becomes_not_ready(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When shooter becomes not ready during shooting, should go back to targeting."""
        # Set up to get to shooting state
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # First execute: idling -> targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting -> shooting
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

        # Turret moves away from target - need engage() each loop
        mock_turret.get_measured_angle_degrees.return_value = 15.0
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "targeting"

    def test_shooting_to_targeting_when_robot_starts_moving(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When robot starts moving during shooting, should go back to targeting."""
        # Set up to get to shooting state
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # First execute: idling -> targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting -> shooting
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

        # Robot starts moving - need engage() each loop
        mock_drivetrain.get_robot_speed.return_value.vx = 0.5
        shooter_sm.engage()
        shooter_sm.execute()

        assert shooter_sm.current_state == "targeting"

    # -------------------------------------------------------------------------
    # Shooting state behavior tests
    # -------------------------------------------------------------------------

    def test_shooting_enables_hopper_and_indexer(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
        mock_hopper,
        mock_indexer,
    ):
        """When in shooting state, hopper and indexer should be enabled."""
        # Set up to get to shooting state
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # First execute: idling -> targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting -> shooting (via next_state_now)
        # Reset mocks so we can check calls from this execute only
        mock_hopper.reset_mock()
        mock_indexer.reset_mock()
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

        # Verify hopper and indexer are enabled during shooting state.
        # Note: targeting calls setEnabled(False) before next_state_now,
        # then shooting calls setEnabled(True). Both run in same execute.
        mock_hopper.setEnabled.assert_any_call(True)
        mock_indexer.setEnabled.assert_any_call(True)

    def test_shooting_tracks_position_and_speed(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When in shooting state, hub tracker should track both position and speed."""
        # Set up to get to shooting state
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # First execute: idling -> targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting -> shooting (via next_state_now)
        # Reset mocks to verify shooting state's calls specifically
        mock_hub_tracker.reset_mock()
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

        # Verify tracking calls from shooting state
        mock_hub_tracker.trackPosition.assert_called_with(True)
        mock_hub_tracker.trackSpeed.assert_called_with(True)

    # -------------------------------------------------------------------------
    # Targeting state behavior tests
    # -------------------------------------------------------------------------

    def test_targeting_disables_hopper_and_indexer(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
        mock_hopper,
        mock_indexer,
    ):
        """When in targeting state, hopper and indexer should be disabled."""
        # Set up shooter not ready so it stays in targeting
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 45.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0

        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        mock_hopper.setEnabled.assert_called_with(False)
        mock_indexer.setEnabled.assert_called_with(False)

    def test_targeting_tracks_position_and_speed(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """When in targeting state, hub tracker should track both position and speed."""
        # Set up shooter not ready so it stays in targeting
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 45.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0

        # First execute: idling -> targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Second execute: targeting state runs and stays in targeting
        # Reset mocks to verify targeting state's calls specifically
        mock_hub_tracker.reset_mock()
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Verify tracking calls from targeting state
        mock_hub_tracker.trackPosition.assert_called_with(True)
        mock_hub_tracker.trackSpeed.assert_called_with(True)

    # -------------------------------------------------------------------------
    # _shooterIsReady tests
    # -------------------------------------------------------------------------

    def test_shooter_is_ready_when_within_all_tolerances(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """Shooter is ready when turret, hood, and flywheel are all within tolerance."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = (
            12.0  # Error = 2 < 3
        )
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = (
            5.5  # Error = 0.5 < 1.5
        )
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = (
            28.0  # Error = 2 < 3
        )

        assert shooter_sm._shooterIsReady() is True

    def test_shooter_is_not_ready_when_turret_out_of_tolerance(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """Shooter is not ready when turret is outside tolerance (3 degrees)."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = (
            5.0  # Error = 5 > 3
        )
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = 5.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = 30.0

        assert shooter_sm._shooterIsReady() is False

    def test_shooter_is_not_ready_when_hood_out_of_tolerance(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """Shooter is not ready when hood is outside tolerance (1.5 degrees)."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = 10.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = (
            8.0  # Error = 3 > 1.5
        )
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = 30.0

        assert shooter_sm._shooterIsReady() is False

    def test_shooter_is_not_ready_when_flywheel_out_of_tolerance(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """Shooter is not ready when flywheel is outside tolerance (3 rps)."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = 10.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = 5.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = (
            25.0  # Error = 5 > 3
        )

        assert shooter_sm._shooterIsReady() is False

    def test_shooter_is_ready_at_exact_tolerance_boundary(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """Shooter is ready when at exact tolerance boundary (<=)."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = (
            13.0  # Error = 3 <= 3
        )
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = (
            6.5  # Error = 1.5 <= 1.5
        )
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = (
            27.0  # Error = 3 <= 3
        )

        assert shooter_sm._shooterIsReady() is True

    # -------------------------------------------------------------------------
    # _robotIsMoving tests
    # -------------------------------------------------------------------------

    def test_robot_is_not_moving_when_stationary(
        self, shooter_sm, mock_drivetrain
    ):
        """Robot is not moving when both vx and vy are zero."""
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        assert shooter_sm._robotIsMoving() is False

    def test_robot_is_moving_when_vx_exceeds_threshold(
        self, shooter_sm, mock_drivetrain
    ):
        """Robot is moving when vx exceeds the threshold."""
        mock_drivetrain.get_robot_speed.return_value.vx = 0.15
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        assert shooter_sm._robotIsMoving() is True

    def test_robot_is_moving_when_vy_exceeds_threshold(
        self, shooter_sm, mock_drivetrain
    ):
        """Robot is moving when vy exceeds the threshold."""
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.15

        assert shooter_sm._robotIsMoving() is True

    def test_robot_is_moving_with_combined_velocity(
        self, shooter_sm, mock_drivetrain
    ):
        """Robot is moving when combined velocity exceeds threshold."""
        # sqrt(0.08^2 + 0.08^2) ≈ 0.113 > 0.1
        mock_drivetrain.get_robot_speed.return_value.vx = 0.08
        mock_drivetrain.get_robot_speed.return_value.vy = 0.08

        assert shooter_sm._robotIsMoving() is True

    def test_robot_is_not_moving_at_threshold(
        self, shooter_sm, mock_drivetrain
    ):
        """Robot is not moving when exactly at threshold (using >)."""
        mock_drivetrain.get_robot_speed.return_value.vx = 0.1
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        # speed = 0.1, threshold is 0.1, using > so should be False
        assert shooter_sm._robotIsMoving() is False

    def test_robot_is_not_moving_with_negative_velocity(
        self, shooter_sm, mock_drivetrain
    ):
        """Robot moving calculation uses squared values so negative velocity still works."""
        mock_drivetrain.get_robot_speed.return_value.vx = -0.15
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        assert shooter_sm._robotIsMoving() is True

    def test_robot_is_moving_with_custom_threshold(
        self, shooter_sm, mock_drivetrain
    ):
        """Custom threshold can be passed to _robotIsMoving."""
        mock_drivetrain.get_robot_speed.return_value.vx = 0.2
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        assert shooter_sm._robotIsMoving(speed_threshold_mps=0.3) is False
        assert shooter_sm._robotIsMoving(speed_threshold_mps=0.1) is True

    # -------------------------------------------------------------------------
    # Edge case tests
    # -------------------------------------------------------------------------

    def test_multiple_state_transitions(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
        mock_drivetrain,
    ):
        """Test a full cycle: idling -> targeting -> shooting -> targeting -> idling."""
        # State machine starts empty, need engage+execute to enter first state
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "idling"

        # Set up not ready to stay in targeting
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 45.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0

        # Go to targeting
        shooter_sm.setDriverWantsFeed(True)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Now make shooter ready
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 0.0
        mock_turret.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 0.0
        mock_hood.get_measured_angle_degrees.return_value = 0.0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 20.0
        mock_flywheel.get_measured_speed_rps.return_value = 20.0
        mock_drivetrain.get_robot_speed.return_value.vx = 0.0
        mock_drivetrain.get_robot_speed.return_value.vy = 0.0

        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "shooting"

        # Robot starts moving - should go to targeting
        mock_drivetrain.get_robot_speed.return_value.vx = 0.5
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "targeting"

        # Driver releases feed button - should go to idling
        shooter_sm.setDriverWantsFeed(False)
        shooter_sm.engage()
        shooter_sm.execute()
        assert shooter_sm.current_state == "idling"

    def test_get_shooter_is_within_with_all_within_tolerance(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """_getShooterIsWithin returns True when all within custom tolerances."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = 15.0  # Error = 5
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = 7.0  # Error = 2
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = 26.0  # Error = 4

        # All within wider tolerances
        result = shooter_sm._getShooterIsWithin(
            turret_tolerance_degrees=6.0,
            hood_tolerance_degrees=3.0,
            flywheel_tolerance_rotations_per_second=5.0,
        )
        assert result is True

    def test_get_shooter_is_within_with_one_out_of_tolerance(
        self,
        shooter_sm,
        mock_hub_tracker,
        mock_turret,
        mock_hood,
        mock_flywheel,
    ):
        """_getShooterIsWithin returns False when one component is out of tolerance."""
        mock_hub_tracker.get_target_turret_angle_degrees.return_value = 10.0
        mock_turret.get_measured_angle_degrees.return_value = 15.0  # Error = 5
        mock_hub_tracker.get_target_hood_angle_degrees.return_value = 5.0
        mock_hood.get_measured_angle_degrees.return_value = 5.0  # Error = 0
        mock_hub_tracker.get_target_flywheel_speed_rps.return_value = 30.0
        mock_flywheel.get_measured_speed_rps.return_value = 30.0  # Error = 0

        # Turret is out of tolerance
        result = shooter_sm._getShooterIsWithin(
            turret_tolerance_degrees=4.0,  # 5 > 4
            hood_tolerance_degrees=1.0,
            flywheel_tolerance_rotations_per_second=1.0,
        )
        assert result is False
