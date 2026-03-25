import logging
import math
import typing

import commands2
import magicbot
import wpilib
import wpimath 
import choreo 
from commands2 import sysid as commands2_sysid
from phoenix6 import hardware, swerve, units, configs, SignalLogger
from wpimath import controller, geometry, kinematics

import constants
from common import alliance, datalog, joystick
from subsystem import drivetrain


class Drivetrain(commands2.Subsystem):
    robot_constants: constants.RobotConstants
    alliance_fetcher: alliance.AllianceFetcher
    data_logger: datalog.DataLogger

    def setup(self) -> None:
        constants = self.robot_constants.drivetrain

        constants_factory: swerve.SwerveModuleConstantsFactory[
            configs.TalonFXConfiguration,
            configs.TalonFXConfiguration,
            configs.CANcoderConfiguration,
        ] = (
            swerve.SwerveModuleConstantsFactory()
            .with_drive_motor_gear_ratio(
                constants.common.drive_motor_gear_ratio
            )
            .with_steer_motor_gear_ratio(
                constants.common.steer_motor_gear_ratio
            )
            .with_coupling_gear_ratio(constants.common.coupling_gear_ratio)
            .with_wheel_radius(constants.common.wheel_radius)
            .with_steer_motor_gains(constants.common.steer_motor_gains)
            .with_drive_motor_gains(constants.common.drive_motor_gains)
            .with_steer_motor_closed_loop_output(
                constants.common.steer_motor_closed_loop_output
            )
            .with_drive_motor_closed_loop_output(
                constants.common.drive_motor_closed_loop_output
            )
            .with_slip_current(constants.common.slip_current)
            .with_speed_at12_volts(constants.common.speed_at12_volts)
            .with_drive_motor_type(constants.common.drive_motor_type)
            .with_steer_motor_type(constants.common.steer_motor_type)
            .with_feedback_source(constants.common.feedback_source)
            .with_drive_motor_initial_configs(
                constants.common.drive_motor_initial_configs
            )
            .with_steer_motor_initial_configs(
                constants.common.steer_motor_initial_configs
            )
            .with_encoder_initial_configs(
                constants.common.encoder_initial_configs
            )
        )

        self.swerve_drive = swerve.SwerveDrivetrain(
            hardware.TalonFX,
            hardware.TalonFX,
            hardware.CANcoder,
            swerve.SwerveDrivetrainConstants()
            .with_can_bus_name(constants.drivetrain.can_bus_name)
            .with_pigeon2_id(constants.drivetrain.pigeon2_id)
            .with_pigeon2_configs(constants.drivetrain.pigeon2_configs),
            [
                constants_factory.create_module_constants(
                    constants.front_left.steer_motor_id,
                    constants.front_left.drive_motor_id,
                    constants.front_left.encoder_id,
                    constants.front_left.encoder_offset,
                    constants.front_left.location_x,
                    constants.front_left.location_y,
                    constants.front_left.drive_motor_inverted,
                    constants.front_left.steer_motor_inverted,
                    constants.front_left.encoder_inverted,
                ),
                constants_factory.create_module_constants(
                    constants.front_right.steer_motor_id,
                    constants.front_right.drive_motor_id,
                    constants.front_right.encoder_id,
                    constants.front_right.encoder_offset,
                    constants.front_right.location_x,
                    constants.front_right.location_y,
                    constants.front_right.drive_motor_inverted,
                    constants.front_right.steer_motor_inverted,
                    constants.front_right.encoder_inverted,
                ),
                constants_factory.create_module_constants(
                    constants.back_left.steer_motor_id,
                    constants.back_left.drive_motor_id,
                    constants.back_left.encoder_id,
                    constants.back_left.encoder_offset,
                    constants.back_left.location_x,
                    constants.back_left.location_y,
                    constants.back_left.drive_motor_inverted,
                    constants.back_left.steer_motor_inverted,
                    constants.back_left.encoder_inverted,
                ),
                constants_factory.create_module_constants(
                    constants.back_right.steer_motor_id,
                    constants.back_right.drive_motor_id,
                    constants.back_right.encoder_id,
                    constants.back_right.encoder_offset,
                    constants.back_right.location_x,
                    constants.back_right.location_y,
                    constants.back_right.drive_motor_inverted,
                    constants.back_right.steer_motor_inverted,
                    constants.back_right.encoder_inverted,
                ),
            ],
        )

        self._x_controller = controller.PIDController(
            constants.trajectory_following.x_kp, 0.0, 0.0
        )
        self._y_controller = controller.PIDController(
            constants.trajectory_following.y_kp, 0.0, 0.0
        )
        self._heading_controller = controller.PIDController(
            constants.trajectory_following.heading_kp, 0.0, 0.0
        )
        # Wrap -180 -> 180 correctly
        self._heading_controller.enableContinuousInput(-math.pi, math.pi)

        self._drive_request = (
            swerve.requests.FieldCentric().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        )

        # When set to True, the brake request will be used instead of the drive
        # request.
        self._brake_enabled = False
        self._brake_request = (
            swerve.requests.SwerveDriveBrake().with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        )

        # When set to True, the auto request will be used instead of the brake
        # or drive requests.
        self._auto_enabled = False
        self._auto_request = (
            swerve.requests.ApplyFieldSpeeds()
            .with_forward_perspective(
                swerve.requests.ForwardPerspectiveValue.BLUE_ALLIANCE
            )
            .with_drive_request_type(
                swerve.SwerveModule.DriveRequestType.VELOCITY
            )
        )

        self._operator_perspective_set = False
        self._maybeSetOperatorPerspectiveForward()

    def execute(self) -> None:
        """Command the drivetrain to the current speeds.

        This method is called at the end of the control loop.
        """
        self._maybeSetOperatorPerspectiveForward()
        if not self._operator_perspective_set:
            self.logger.warning("Driving without operator perspective set")

        if self._auto_enabled:
            self.swerve_drive.set_control(self._auto_request)
        elif self._brake_enabled:
            self.swerve_drive.set_control(self._brake_request)
        else:
            self.swerve_drive.set_control(self._drive_request)

        self._logData()

    def setSpeeds(
        self,
        command: joystick.DriveCommand,
    ) -> None:
        """Set the drivetrain's target speeds."""
        self._drive_request.with_velocity_x(command.vx).with_velocity_y(
            command.vy
        ).with_rotational_rate(command.omega)

    def followTrajectorySample(self, sample: choreo.SwerveSample) -> None:
        """Follow a Choreo trajectory sample.

        Add corrections to the sample's velocities to stay on track. Use field
        centric control requests, but always relative to blue alliance origin,
        so that we don't have to negate the speeds.

        Args:
            sample:
                The Choreo trajectory sample to follow.
        """
        pose = self.swerve_drive.get_state().pose
        self._auto_request.with_speeds(
            kinematics.ChassisSpeeds(
                sample.vx + self._x_controller.calculate(pose.X(), sample.x),
                sample.vy + self._y_controller.calculate(pose.Y(), sample.y),
                sample.omega
                + self._heading_controller.calculate(
                    pose.rotation().radians(), sample.heading
                ),
            )
        )

    def stop(self) -> None:
        """Stop the drivetrain."""
        self._auto_request.with_speeds(kinematics.ChassisSpeeds(0, 0, 0))
        self._drive_request.with_velocity_x(0).with_velocity_y(
            0
        ).with_rotational_rate(0)

    def setPose(self, pose: geometry.Pose2d) -> None:
        """Hard reset the robot's pose estimate."""
        self.swerve_drive.reset_pose(pose)

    def setBrakeEnabled(self, value: bool) -> None:
        self._brake_enabled = value

    def setAutoEnabled(self, value: bool) -> None:
        self._auto_enabled = value

    def _maybeSetOperatorPerspectiveForward(self) -> None:
        if self._operator_perspective_set:
            return
        if self.alliance_fetcher.hasAlliance():
            self.logger.info(f"Setting operator perspective for {alliance}")
            self.swerve_drive.set_operator_perspective_forward(
                drivetrain.constants.RED_ALLIANCE_PERSPECTIVE_ROTATION
                if self.alliance_fetcher.isRedAlliance()
                else drivetrain.constants.BLUE_ALLIANCE_PERSPECTIVE_ROTATION
            )
            self._operator_perspective_set = True

    @magicbot.feedback
    def get_robot_pose(self) -> geometry.Pose2d:
        return self.swerve_drive.get_state().pose

    def robotSpeeds(self) -> wpimath.kinematics.ChassisSpeeds:
        return self.swerve_drive.get_state().speeds

    def rawYawDegrees(self) -> units.degree:
        return wpimath.inputModulus(
            self.swerve_drive.pigeon2.get_yaw().value, -180.0, 180.0
        )

    def rawPitchDegrees(self) -> units.degree:
        return wpimath.inputModulus(
            self.swerve_drive.pigeon2.get_pitch().value, -180.0, 180.0
        )

    def rawRollDegrees(self) -> units.degree:
        return wpimath.inputModulus(
            self.swerve_drive.pigeon2.get_roll().value, -180.0, 180.0
        )

    def estimatedYawDegrees(self) -> units.degree:
        return self.swerve_drive.get_state().pose.rotation().degrees()

    def frontLeftDriveSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(0)
            .drive_motor.get_supply_current()
            .value
        )

    def frontLeftDriveStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(0)
            .drive_motor.get_stator_current()
            .value
        )

    def frontLeftSteerSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(0)
            .steer_motor.get_supply_current()
            .value
        )

    def frontLeftSteerStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(0)
            .steer_motor.get_stator_current()
            .value
        )

    def frontRightDriveSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(1)
            .drive_motor.get_supply_current()
            .value
        )

    def frontRightDriveStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(1)
            .drive_motor.get_stator_current()
            .value
        )

    def frontRightSteerSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(1)
            .steer_motor.get_supply_current()
            .value
        )

    def frontRightSteerStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(1)
            .steer_motor.get_stator_current()
            .value
        )

    def backLeftDriveSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(2)
            .drive_motor.get_supply_current()
            .value
        )

    def backLeftDriveStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(2)
            .drive_motor.get_stator_current()
            .value
        )

    def backLeftSteerSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(2)
            .steer_motor.get_supply_current()
            .value
        )

    def backLeftSteerStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(2)
            .steer_motor.get_stator_current()
            .value
        )

    def backRightDriveSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(3)
            .drive_motor.get_supply_current()
            .value
        )

    def backRightDriveStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(3)
            .drive_motor.get_stator_current()
            .value
        )

    def backRightSteerSupplyCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(3)
            .steer_motor.get_supply_current()
            .value
        )

    def backRightSteerStatorCurrent(self) -> units.ampere:
        return (
            self.swerve_drive.get_module(3)
            .steer_motor.get_stator_current()
            .value
        )

    def _logData(self) -> None:
        self.data_logger.logDouble(
            "/components/drivetrain/estimated_yaw_degrees",
            self.estimatedYawDegrees(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/raw_yaw_degrees", self.rawYawDegrees()
        )
        self.data_logger.logDouble(
            "/components/drivetrain/raw_pitch_degrees", self.rawPitchDegrees()
        )
        self.data_logger.logDouble(
            "/components/drivetrain/raw_roll_degrees", self.rawRollDegrees()
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_left_drive_supply_current",
            self.frontLeftDriveSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_left_drive_stator_current",
            self.frontLeftDriveStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_left_steer_supply_current",
            self.frontLeftSteerSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_left_steer_stator_current",
            self.frontLeftSteerStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_right_drive_supply_current",
            self.frontRightDriveSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_right_drive_stator_current",
            self.frontRightDriveStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_right_steer_supply_current",
            self.frontRightSteerSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/front_right_steer_stator_current",
            self.frontRightSteerStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_left_drive_supply_current",
            self.backLeftDriveSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_left_drive_stator_current",
            self.backLeftDriveStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_left_steer_supply_current",
            self.backLeftSteerSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_left_steer_stator_current",
            self.backLeftSteerStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_right_drive_supply_current",
            self.backRightDriveSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_right_drive_stator_current",
            self.backRightDriveStatorCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_right_steer_supply_current",
            self.backRightSteerSupplyCurrent(),
        )
        self.data_logger.logDouble(
            "/components/drivetrain/back_right_steer_stator_current",
            self.backRightSteerStatorCurrent(),
        )


class DrivetrainTuner:
    """Component for tuning the drivetrain."""

    drivetrain: Drivetrain

    translation_quasistatic = magicbot.tunable(False)
    translation_dynamic = magicbot.tunable(False)
    rotation_quasistatic = magicbot.tunable(False)
    rotation_dynamic = magicbot.tunable(False)
    steer_quasistatic = magicbot.tunable(False)
    steer_dynamic = magicbot.tunable(False)

    reverse = magicbot.tunable(False)

    # PID gains for trajectory following.
    trajectory_x_kp = magicbot.tunable(0.0)
    trajectory_x_ki = magicbot.tunable(0.0)
    trajectory_x_kd = magicbot.tunable(0.0)
    trajectory_y_kp = magicbot.tunable(0.0)
    trajectory_y_ki = magicbot.tunable(0.0)
    trajectory_y_kd = magicbot.tunable(0.0)
    trajectory_heading_kp = magicbot.tunable(0.0)
    trajectory_heading_ki = magicbot.tunable(0.0)
    trajectory_heading_kd = magicbot.tunable(0.0)

    def setup(self) -> None:
        self._last_tq = self.translation_quasistatic
        self._last_td = self.translation_dynamic
        self._last_rq = self.rotation_quasistatic
        self._last_rd = self.rotation_dynamic
        self._last_sq = self.steer_quasistatic
        self._last_sd = self.steer_dynamic

        self._translation_request = swerve.requests.SysIdSwerveTranslation()
        self._rotation_request = swerve.requests.SysIdSwerveRotation()
        self._steer_request = swerve.requests.SysIdSwerveSteerGains()

        self._sysid_config = commands2_sysid.SysIdRoutine.Config(
            stepVoltage=2.0,
            timeout=2.0,
            recordState=lambda state: SignalLogger.write_string(
                "state", wpilib.sysid.SysIdRoutineLog.stateEnumToString(state)
            ),
        )

        self._sysid_translation = commands2_sysid.SysIdRoutine(
            self._sysid_config,
            commands2_sysid.SysIdRoutine.Mechanism(
                lambda volts: self.drivetrain.swerve_drive.set_control(
                    self._translation_request.with_volts(volts)
                ),
                lambda log: None,
                self.drivetrain,
                "Drivetrain",
            ),
        )

        self._sysid_rotation = commands2_sysid.SysIdRoutine(
            self._sysid_config,
            commands2_sysid.SysIdRoutine.Mechanism(
                lambda rotational_rate: self.drivetrain.swerve_drive.set_control(
                    self._rotation_request.with_rotational_rate(rotational_rate)
                ),
                lambda log: None,
                self.drivetrain,
                "Drivetrain",
            ),
        )

        self._sysid_steer = commands2_sysid.SysIdRoutine(
            self._sysid_config,
            commands2_sysid.SysIdRoutine.Mechanism(
                lambda volts: self.drivetrain.swerve_drive.set_control(
                    self._steer_request.with_volts(volts)
                ),
                lambda log: None,
                self.drivetrain,
                "Drivetrain",
            ),
        )

        self._scheduler = commands2.CommandScheduler.getInstance()

    def on_enable(self) -> None:
        self._scheduler.enable()

    def on_disable(self) -> None:
        self._scheduler.disable()
        SignalLogger.stop()

    def execute(self) -> None:
        self.drivetrain._x_controller.setPID(
            self.trajectory_x_kp, self.trajectory_x_ki, self.trajectory_x_kd
        )
        self.drivetrain._y_controller.setPID(
            self.trajectory_y_kp, self.trajectory_y_ki, self.trajectory_y_kd
        )
        self.drivetrain._heading_controller.setPID(
            self.trajectory_heading_kp,
            self.trajectory_heading_ki,
            self.trajectory_heading_kd,
        )

        if (
            sum(
                [
                    self.translation_quasistatic,
                    self.translation_dynamic,
                    self.rotation_quasistatic,
                    self.rotation_dynamic,
                    self.steer_quasistatic,
                    self.steer_dynamic,
                ]
            )
            > 1
        ):
            self.logger.warning(
                "Cannot apply multiple sysid routines simultaneously"
            )
            self._updateState()
            return

        direction = (
            commands2_sysid.SysIdRoutine.Direction.kReverse
            if self.reverse
            else commands2_sysid.SysIdRoutine.Direction.kForward
        )
        # Only schedule a sysid command on a rising edge.
        if self.translation_quasistatic and not self._last_tq:
            self.sysIdTranslationQuasistatic(direction)
        if self.translation_dynamic and not self._last_td:
            self.sysIdTranslationDynamic(direction)
        if self.rotation_quasistatic and not self._last_rq:
            self.sysIdRotationQuasistatic(direction)
        if self.rotation_dynamic and not self._last_rd:
            self.sysIdRotationDynamic(direction)
        if self.steer_quasistatic and not self._last_sq:
            self.sysIdSteerQuasistatic(direction)
        if self.steer_dynamic and not self._last_sd:
            self.sysIdSteerDynamic(direction)

        self._updateState()
        self._scheduler.run()

    def _updateState(self) -> None:
        self._last_tq = self.translation_quasistatic
        self._last_td = self.translation_dynamic
        self._last_rq = self.rotation_quasistatic
        self._last_rd = self.rotation_dynamic
        self._last_sq = self.steer_quasistatic
        self._last_sd = self.steer_dynamic

    def sysIdTranslationQuasistatic(
        self, direction: commands2_sysid.SysIdRoutine.Direction
    ) -> None:
        self._scheduler.cancelAll()
        self._scheduler.schedule(self._sysid_translation.quasistatic(direction))

    def sysIdTranslationDynamic(
        self, direction: commands2_sysid.SysIdRoutine.Direction
    ) -> None:
        self._scheduler.cancelAll()
        self._scheduler.schedule(self._sysid_translation.dynamic(direction))

    def sysIdRotationQuasistatic(
        self, direction: commands2_sysid.SysIdRoutine.Direction
    ) -> None:
        self._scheduler.cancelAll()
        self._scheduler.schedule(self._sysid_rotation.quasistatic(direction))

    def sysIdRotationDynamic(
        self, direction: commands2_sysid.SysIdRoutine.Direction
    ) -> None:
        self._scheduler.cancelAll()
        self._scheduler.schedule(self._sysid_rotation.dynamic(direction))

    def sysIdSteerQuasistatic(
        self, direction: commands2_sysid.SysIdRoutine.Direction
    ) -> None:
        self._scheduler.cancelAll()
        self._scheduler.schedule(self._sysid_steer.quasistatic(direction))

    def sysIdSteerDynamic(
        self, direction: commands2_sysid.SysIdRoutine.Direction
    ) -> None:
        self._scheduler.cancelAll()
        self._scheduler.schedule(self._sysid_steer.dynamic(direction))
