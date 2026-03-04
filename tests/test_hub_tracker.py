import types

import pytest
from wpimath import geometry

import subsystem.shooter.hub_tracker as hub_tracker


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
    tracker.drivetrain = mocker.Mock()
    yaw_rate_signal = mocker.Mock()
    yaw_rate_signal.value = yaw_rate_degrees_per_second
    tracker.drivetrain.pigeon2.get_angular_velocity_z_world.return_value = (
        yaw_rate_signal
    )
    tracker.drivetrain.get_state.return_value = types.SimpleNamespace(
        pose=robot_pose
    )
    tracker.turret = mocker.Mock()
    tracker.setup()
    return tracker


def _hub_relative_position(dx: float, dy: float) -> geometry.Translation2d:
    """Return a field position offset from the hub center."""
    return geometry.Translation2d(
        hub_tracker.HUB_TO_FIELD_X + dx,
        hub_tracker.HUB_TO_FIELD_Y + dy,
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
        hub_tracker.HUB_TO_FIELD_X,
        hub_tracker.HUB_TO_FIELD_Y,
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
        is tracker.drivetrain.pigeon2.get_angular_velocity_z_world.return_value
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
    tracker = _make_tracker(
        mocker,
        robot_pose=geometry.Pose2d(),
        yaw_rate_degrees_per_second=0.0,
    )
    mocker.patch.object(
        tracker,
        "get_turret_target_angle_degrees",
        return_value=17.5,
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
    tracker = _make_tracker(
        mocker,
        robot_pose=geometry.Pose2d(),
        min_angle=-30.0,
        max_angle=30.0,
        yaw_rate_degrees_per_second=yaw_rate_degrees_per_second,
    )
    mocker.patch.object(
        tracker,
        "get_turret_target_angle_degrees",
        return_value=requested_angle,
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
    tracker = _make_tracker(
        mocker,
        robot_pose=geometry.Pose2d(),
        min_angle=-30.0,
        max_angle=30.0,
        yaw_rate_degrees_per_second=yaw_rate_degrees_per_second,
    )
    mocker.patch.object(
        tracker,
        "get_turret_target_angle_degrees",
        return_value=requested_angle,
    )

    tracker.execute()

    tracker._yaw_rate_signal.refresh.assert_called_once()
    tracker.turret.setPosition.assert_called_once()
    (commanded_angle,) = tracker.turret.setPosition.call_args.args
    assert commanded_angle == pytest.approx(expected_command)


def test_execute_updates_compensation_across_control_loops(mocker) -> None:
    """Each execute loop should refresh yaw-rate and recompute compensation."""
    tracker = _make_tracker(
        mocker,
        robot_pose=geometry.Pose2d(),
        yaw_rate_degrees_per_second=50.0,
    )
    mocker.patch.object(
        tracker,
        "get_turret_target_angle_degrees",
        return_value=10.0,
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


def test_get_turret_target_angle_degrees_is_relative_to_heading(mocker) -> None:
    """Target angle feedback is computed relative to current turret heading."""
    tracker = _make_tracker(mocker, geometry.Pose2d())
    tracker._hub_position = geometry.Translation2d(1.0, 0.0)
    tracker._turret_field_pose = geometry.Pose2d(
        geometry.Translation2d(),
        geometry.Rotation2d.fromDegrees(90.0),
    )

    assert tracker.get_turret_target_angle_degrees() == pytest.approx(-90.0)
