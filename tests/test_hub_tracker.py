import types

import pytest
from wpimath import geometry

import subsystem.shooter.hub_tracker as hub_tracker


class TestShotTable:
    """Unit tests for the ShotTable lookup and linear interpolation logic."""

    def test_clamps_below_minimum(self):
        """Values at or below the first table entry returns the first row."""
        hood, flywheel = hub_tracker.ShotTable.get(0.0)
        assert hood == 0.5
        assert flywheel == 26.0

        hood, flywheel = hub_tracker.ShotTable.get(1.0)
        assert hood == 0.5
        assert flywheel == 26.0

    def test_clamps_above_maximum(self):
        """Values at or above the last table entry returns the last row."""
        hood, flywheel = hub_tracker.ShotTable.get(10.0)
        assert hood == 7.5
        assert flywheel == 35.0

        hood, flywheel = hub_tracker.ShotTable.get(5.0)
        assert hood == 7.5
        assert flywheel == 35.0

    def test_exact_table_points(self):
        """Every exact distance that exists in the table returns the exact stored values (no interpolation)."""
        assert hub_tracker.ShotTable.get(1.75) == (0.5, 26.0)
        assert hub_tracker.ShotTable.get(2.25) == (3.0, 29.0)
        assert hub_tracker.ShotTable.get(2.75) == (4.0, 30.0)
        assert hub_tracker.ShotTable.get(3.25) == (5.0, 31.0)
        assert hub_tracker.ShotTable.get(3.75) == (6.5, 33.0)
        assert hub_tracker.ShotTable.get(4.25) == (7.0, 34.0)
        assert hub_tracker.ShotTable.get(4.75) == (7.5, 35.0)

    @pytest.mark.parametrize(
        "distance, expected_hood, expected_flywheel",
        [(2.5, 3.5, 29.5), (3.5, 5.75, 32.0)],
    )
    def test_linear_interpolation(
        self, distance, expected_hood, expected_flywheel
    ):
        """Linear interpolation produces the mathematically correct value between any two table rows."""
        hood, flywheel = hub_tracker.ShotTable.get(distance)
        assert hood == pytest.approx(expected_hood, abs=1e-9)
        assert flywheel == pytest.approx(expected_flywheel, abs=1e-9)

    def test_interpolation_near_edges(self):
        """Interpolation works exactly right next to the first and last intervals (no off-by-one bugs)."""
        # Just inside the first interval
        hood, flywheel = hub_tracker.ShotTable.get(1.76)
        assert hood == pytest.approx(0.55, abs=1e-9)
        assert flywheel == pytest.approx(26.06, abs=1e-9)

        # Just inside the last interval
        hood, flywheel = hub_tracker.ShotTable.get(4.74)
        assert hood == pytest.approx(7.49, abs=1e-9)
        assert flywheel == pytest.approx(34.98, abs=1e-9)

    def test_table_and_distances_are_equal_length(self):
        """_TABLE and _DISTANCES are of equal length.

        This is important as we use _DISTANCES to interpolate and _TABLES for
        lookup.
        """
        assert len(hub_tracker.ShotTable._TABLE) == len(
            hub_tracker.ShotTable._DISTANCES
        )


def _make_tracker(
    mocker,
    robot_pose: geometry.Pose2d,
    min_angle: float = -180.0,
    max_angle: float = 180.0,
    yaw_rate_degrees_per_second: float = 0.0,
) -> hub_tracker.HubTracker:
    """Build a HubTracker with mocked dependencies and turret limits."""
    tracker = hub_tracker.HubTracker()
    tracker.robot_constants = types.SimpleNamespace(
        shooter=types.SimpleNamespace(
            turret=types.SimpleNamespace(
                min_angle=min_angle,
                max_angle=max_angle,
            )
        )
    )
    tracker.alliance_fetcher = mocker.Mock()
    tracker.alliance_fetcher.getAlliance.return_value = None
    tracker.drivetrain = mocker.Mock()
    yaw_rate_signal = mocker.Mock()
    yaw_rate_signal.value = yaw_rate_degrees_per_second
    tracker.drivetrain.swerve_drive.pigeon2.get_angular_velocity_z_world.return_value = (
        yaw_rate_signal
    )
    tracker.drivetrain.swerve_drive.get_state.return_value = (
        types.SimpleNamespace(pose=robot_pose)
    )
    tracker.flywheel = mocker.Mock()
    tracker.hood = mocker.Mock()
    tracker.turret = mocker.Mock()
    tracker.setup()
    return tracker


def _hub_relative_position(dx: float, dy: float) -> geometry.Translation2d:
    """Return a field position offset from the hub center."""
    return geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X + dx,
        hub_tracker.BLUE_HUB_TO_FIELD_Y + dy,
    )


def _robot_pose_with_turret_at(
    turret_position: geometry.Translation2d,
    robot_yaw_degrees: float,
) -> geometry.Pose2d:
    """Construct a robot pose that places the turret at turret_position."""
    robot_rotation = geometry.Rotation2d.fromDegrees(robot_yaw_degrees)
    turret_offset = geometry.Translation2d(
        hub_tracker.TURRET_TO_ROBOT_X,
        hub_tracker.TURRET_TO_ROBOT_Y,
    ).rotateBy(robot_rotation)
    return geometry.Pose2d(turret_position - turret_offset, robot_rotation)


def test_setup_initializes_known_transforms(mocker) -> None:
    """setup initializes robot-to-turret and field-to-hub geometry."""
    tracker = _make_tracker(mocker, geometry.Pose2d())

    expected_turret_offset = geometry.Translation2d(
        hub_tracker.TURRET_TO_ROBOT_X,
        hub_tracker.TURRET_TO_ROBOT_Y,
    )
    expected_hub_position = geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X,
        hub_tracker.BLUE_HUB_TO_FIELD_Y,
    )

    offset_error = (
        expected_turret_offset
        - tracker._robot_to_turret_transform.translation()
    )
    hub_error = expected_hub_position - tracker._hub_position

    assert offset_error.norm() == pytest.approx(0.0, abs=1e-9)
    assert hub_error.norm() == pytest.approx(0.0, abs=1e-9)
    assert (
        tracker._yaw_rate_signal
        is tracker.drivetrain.swerve_drive.pigeon2.get_angular_velocity_z_world.return_value
    )


@pytest.mark.parametrize(
    (
        "robot_pose",
        "yaw_rate_degrees_per_second",
        "expected_command_angle",
    ),
    [
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(-2.0, 0.0),
                robot_yaw_degrees=45.0,
            ),
            50.0,
            -46.0,
            id="behind_hub_yaw_45",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(2.0, 0.0),
                robot_yaw_degrees=180.0,
            ),
            -50.0,
            1.0,
            id="in_front_of_hub_yaw_180",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(0.0, 2.0),
                robot_yaw_degrees=0.0,
            ),
            100.0,
            -92.0,
            id="left_of_hub_yaw_0",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(0.0, -2.0),
                robot_yaw_degrees=0.0,
            ),
            -100.0,
            92.0,
            id="right_of_hub_yaw_0",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(-2.0, 2.0),
                robot_yaw_degrees=-90.0,
            ),
            150.0,
            42.0,
            id="southwest_of_hub_yaw_neg_90",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(2.0, 2.0),
                robot_yaw_degrees=-90.0,
            ),
            -150.0,
            -42.0,
            id="northwest_of_hub_yaw_neg_90",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(2.0, -2.0),
                robot_yaw_degrees=90.0,
            ),
            200.0,
            41.0,
            id="northeast_of_hub_yaw_90",
        ),
        pytest.param(
            _robot_pose_with_turret_at(
                _hub_relative_position(-2.0, -2.0),
                robot_yaw_degrees=90.0,
            ),
            -200.0,
            -41.0,
            id="southeast_of_hub_yaw_90",
        ),
    ],
)
def test_execute_updates_turret_pose_and_commands_angle(
    mocker,
    robot_pose: geometry.Pose2d,
    yaw_rate_degrees_per_second: float,
    expected_command_angle: float,
) -> None:
    """execute computes turret field pose and commands the tracking angle."""
    tracker = _make_tracker(
        mocker,
        robot_pose,
        yaw_rate_degrees_per_second=yaw_rate_degrees_per_second,
    )

    expected_turret_pose = robot_pose.transformBy(
        tracker._robot_to_turret_transform
    )

    tracker.execute()

    translation_error = (
        expected_turret_pose.translation()
        - tracker._turret_field_pose.translation()
    )
    rotation_error = (
        expected_turret_pose.rotation() - tracker._turret_field_pose.rotation()
    ).degrees()

    tracker._yaw_rate_signal.refresh.assert_called_once()
    tracker.turret.setPosition.assert_called_once()
    (commanded_angle,) = tracker.turret.setPosition.call_args.args

    assert commanded_angle == pytest.approx(expected_command_angle, abs=1e-9)
    assert translation_error.norm() == pytest.approx(0.0, abs=1e-9)
    assert rotation_error == pytest.approx(0.0, abs=1e-9)


def test_execute_zero_yaw_rate_does_not_offset_target_angle(mocker) -> None:
    """Zero yaw-rate should produce no predictive offset in commanded angle."""
    blue_hub = geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X, hub_tracker.BLUE_HUB_TO_FIELD_Y
    )
    turret_pos = blue_hub - geometry.Translation2d(1.0, 0.0).rotateBy(
        geometry.Rotation2d.fromDegrees(17.5)
    )
    tracker = _make_tracker(
        mocker,
        robot_pose=_robot_pose_with_turret_at(
            turret_pos, robot_yaw_degrees=0.0
        ),
        yaw_rate_degrees_per_second=0.0,
    )

    tracker.execute()

    tracker._yaw_rate_signal.refresh.assert_called_once()
    tracker.turret.setPosition.assert_called_once()
    (commanded_angle,) = tracker.turret.setPosition.call_args.args
    assert commanded_angle == pytest.approx(17.5)


@pytest.mark.parametrize(
    (
        "requested_angle",
        "yaw_rate_degrees_per_second",
        "expected_command",
    ),
    [
        (120.0, 50.0, 30.0),
        (-120.0, -50.0, -30.0),
    ],
)
def test_execute_clamps_target_angle_to_limits(
    mocker,
    requested_angle: float,
    yaw_rate_degrees_per_second: float,
    expected_command: float,
) -> None:
    """execute clamps commanded turret angle to configured min/max limits."""
    blue_hub = geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X, hub_tracker.BLUE_HUB_TO_FIELD_Y
    )
    turret_pos = blue_hub - geometry.Translation2d(1.0, 0.0).rotateBy(
        geometry.Rotation2d.fromDegrees(requested_angle)
    )
    tracker = _make_tracker(
        mocker,
        robot_pose=_robot_pose_with_turret_at(
            turret_pos, robot_yaw_degrees=0.0
        ),
        min_angle=-30.0,
        max_angle=30.0,
        yaw_rate_degrees_per_second=yaw_rate_degrees_per_second,
    )

    tracker.execute()

    tracker._yaw_rate_signal.refresh.assert_called_once()
    tracker.turret.setPosition.assert_called_once()
    (commanded_angle,) = tracker.turret.setPosition.call_args.args
    assert commanded_angle == pytest.approx(expected_command)


@pytest.mark.parametrize(
    (
        "requested_angle",
        "yaw_rate_degrees_per_second",
        "expected_command",
    ),
    [
        (29.0, -100.0, 30.0),
        (-29.0, 100.0, -30.0),
    ],
)
def test_execute_compensation_applied_before_clamping(
    mocker,
    requested_angle: float,
    yaw_rate_degrees_per_second: float,
    expected_command: float,
) -> None:
    """Compensation happens before limit clamping and can push in-range targets out."""
    blue_hub = geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X, hub_tracker.BLUE_HUB_TO_FIELD_Y
    )
    turret_pos = blue_hub - geometry.Translation2d(1.0, 0.0).rotateBy(
        geometry.Rotation2d.fromDegrees(requested_angle)
    )
    tracker = _make_tracker(
        mocker,
        robot_pose=_robot_pose_with_turret_at(
            turret_pos, robot_yaw_degrees=0.0
        ),
        min_angle=-30.0,
        max_angle=30.0,
        yaw_rate_degrees_per_second=yaw_rate_degrees_per_second,
    )

    tracker.execute()

    tracker._yaw_rate_signal.refresh.assert_called_once()
    tracker.turret.setPosition.assert_called_once()
    (commanded_angle,) = tracker.turret.setPosition.call_args.args
    assert commanded_angle == pytest.approx(expected_command)


def test_execute_updates_compensation_across_control_loops(mocker) -> None:
    """Each execute loop should refresh yaw-rate and recompute compensation."""
    blue_hub = geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X, hub_tracker.BLUE_HUB_TO_FIELD_Y
    )
    turret_pos = blue_hub - geometry.Translation2d(1.0, 0.0).rotateBy(
        geometry.Rotation2d.fromDegrees(10.0)
    )
    tracker = _make_tracker(
        mocker,
        robot_pose=_robot_pose_with_turret_at(
            turret_pos, robot_yaw_degrees=0.0
        ),
        yaw_rate_degrees_per_second=50.0,
    )

    tracker.execute()
    tracker._yaw_rate_signal.value = -50.0
    tracker.execute()

    assert tracker._yaw_rate_signal.refresh.call_count == 2
    assert tracker.turret.setPosition.call_count == 2
    first_commanded_angle = tracker.turret.setPosition.call_args_list[0].args[0]
    second_commanded_angle = tracker.turret.setPosition.call_args_list[1].args[
        0
    ]
    assert first_commanded_angle == pytest.approx(9.0)
    assert second_commanded_angle == pytest.approx(11.0)


def test_get_turret_distance_from_hub_meters(mocker) -> None:
    """Distance feedback returns Euclidean distance from turret to hub."""
    tracker = _make_tracker(mocker, geometry.Pose2d())
    tracker._hub_position = geometry.Translation2d(4.0, 6.0)
    tracker._turret_field_pose = geometry.Pose2d(
        geometry.Translation2d(1.0, 2.0),
        geometry.Rotation2d(),
    )

    assert tracker.get_turret_distance_from_hub_meters() == pytest.approx(5.0)


def test_compute_target_turret_angle_degrees_is_relative_to_heading(
    mocker,
) -> None:
    """Computed target angle is relative to current turret heading."""
    tracker = _make_tracker(mocker, geometry.Pose2d())
    blue_hub = geometry.Translation2d(
        hub_tracker.BLUE_HUB_TO_FIELD_X, hub_tracker.BLUE_HUB_TO_FIELD_Y
    )
    tracker._turret_field_pose = geometry.Pose2d(
        blue_hub - geometry.Translation2d(1.0, 0.0),
        geometry.Rotation2d.fromDegrees(90.0),
    )

    assert tracker._computeTargetTurretAngleDegrees() == pytest.approx(-90.0)


def test_get_target_turret_angle_degrees_returns_cached_value(mocker) -> None:
    """Target turret getter returns the cached target command value."""
    tracker = _make_tracker(mocker, geometry.Pose2d())
    tracker._target_turret_angle_degrees = 12.34

    assert tracker.get_target_turret_angle_degrees() == pytest.approx(12.34)
