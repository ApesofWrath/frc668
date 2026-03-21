import magicbot
import phoenix6

import constants
from subsystem import drivetrain, shooter


class Turret:
    DEGREES_TO_ROTATIONS = 1.0 / 360.0
    ROTATIONS_TO_DEGREES = 360.0

    DEGREES_TO_ROTATIONS = 1.0 / 360.0
    ROTATIONS_TO_DEGREES = 360.0

    robot_constants: constants.RobotConstants
    turret_motor: phoenix6.hardware.TalonFX
    turret_encoder: phoenix6.hardware.CANcoder
    drivetrain: drivetrain.Drivetrain

    def setup(self) -> None:
        """Set up initial state for the turret.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # Target position for the turret, in degrees. Counter clockwise is
        # positive.
        self._turret_postion_degrees = 0.0

        # Target rotation speed for the turret, in degrees per second. Counter
        # clockwise is positive.
        self._turret_velocity_degrees_per_second = 0.0
        self._is_velocity_controlled = False

        turret_constants: shooter.TurretConstants = (
            self.robot_constants.shooter.turret
        )
        self.turret_motor_configs = (
            phoenix6.configs.TalonFXConfiguration()
            .with_feedback(
                phoenix6.configs.FeedbackConfigs()
                .with_feedback_sensor_source(
                    phoenix6.signals.FeedbackSensorSourceValue.FUSED_CANCODER
                )
                .with_feedback_remote_sensor_id(turret_constants.encoder_can_id)
                .with_sensor_to_mechanism_ratio(
                    turret_constants.sensor_to_mechanism_ratio
                )
                .with_rotor_to_sensor_ratio(
                    turret_constants.rotor_to_sensor_ratio
                )
            )
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    turret_constants.motor_inverted
                )
            )
            .with_slot0(
                phoenix6.configs.Slot0Configs()
                .with_k_s(turret_constants.position_k_s)
                .with_k_v(turret_constants.position_k_v)
                .with_k_a(turret_constants.position_k_a)
                .with_k_p(turret_constants.position_k_p)
                .with_k_i(turret_constants.position_k_i)
                .with_k_d(turret_constants.position_k_d)
            )
            .with_slot1(
                phoenix6.configs.Slot1Configs()
                .with_k_s(turret_constants.velocity_k_s)
                .with_k_v(turret_constants.velocity_k_v)
                .with_k_a(turret_constants.velocity_k_a)
                .with_k_p(turret_constants.velocity_k_p)
                .with_k_i(turret_constants.velocity_k_i)
                .with_k_d(turret_constants.velocity_k_d)
            )
            .with_motion_magic(
                phoenix6.configs.MotionMagicConfigs()
                .with_motion_magic_cruise_velocity(
                    turret_constants.motion_magic_cruise_velocity
                )
                .with_motion_magic_acceleration(
                    turret_constants.motion_magic_acceleration
                )
                .with_motion_magic_jerk(turret_constants.motion_magic_jerk)
            )
            .with_current_limits(
                phoenix6.configs.CurrentLimitsConfigs()
                .with_supply_current_limit(
                    turret_constants.supply_current_limit
                )
                .with_supply_current_limit_enable(True)
            )
        )
        result = self.turret_motor.configurator.apply(self.turret_motor_configs)
        if not result.is_ok():
            self.logger.error("Failed to apply configs to turret motor")

        result = self.turret_encoder.configurator.apply(
            phoenix6.configs.CANcoderConfiguration().with_magnet_sensor(
                phoenix6.configs.MagnetSensorConfigs()
                .with_sensor_direction(turret_constants.encoder_direction)
                .with_absolute_sensor_discontinuity_point(
                    turret_constants.absolute_sensor_discontinuity_point
                )
                .with_magnet_offset(turret_constants.magnet_offset)
            )
        )
        if not result.is_ok():
            self.logger.error("Failed to apply configs to turret encoder")

        # Initial Motion Magic request (position expressed in rotations)
        self._request = phoenix6.controls.MotionMagicVoltage(
            self._turret_postion_degrees * self.DEGREES_TO_ROTATIONS
        ).with_slot(0)
        # Feedforward constant for computing the feedforward voltage to use with
        # the control request each loop, to compensate for robot's yaw rate.
        self._motion_magic_feed_forward = (
            turret_constants.motion_magic_feed_forward
        )
        # Raw yaw rate of the robot (and the turret) from the external IMU.
        self._yaw_rate_signal: phoenix6.status_signal.StatusSignal[
            phoenix6.units.degrees_per_second
        ] = self.drivetrain.swerve_drive.pigeon2.get_angular_velocity_z_world()
        # Raw position from the external encoder.
        self._encoder_position_signal: phoenix6.status_signal.StatusSignal[
            phoenix6.units.rotation
        ] = self.turret_encoder.get_position()

        # Our sensor to mechanism ratio is 10. This means if our turret starts
        # anywhere in the position range [-18, 18) degrees (assuming 0 is
        # straight ahead), we can use the absolute offset from the encoder to
        # set both the encoder's and motor's position to it so both of their
        # zeros also point straight ahead.
        absolute_position = self.turret_encoder.get_absolute_position().value
        self.logger.info(
            f"Setting turret position to {absolute_position} rotations"
        )
        result = self.turret_encoder.set_position(absolute_position)
        if not result.is_ok():
            self.logger.error("Failed to set position on turret encoder")
        result = self.turret_motor.set_position(absolute_position)
        if not result.is_ok():
            self.logger.error("Failed to set position on turret motor")

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self._encoder_position_signal.refresh()
        if self._is_velocity_controlled:
            self.turret_motor.set_control(
                phoenix6.controls.VelocityVoltage(
                    self._turret_velocity_degrees_per_second
                    * self.DEGREES_TO_ROTATIONS
                ).with_slot(1)
            )
        else:
            # Calculate a feedforward to provide as an assist to the position
            # controller in counteracting the robot's yaw rate, so it tracks the
            # target with considerably less lag.
            self._yaw_rate_signal.refresh()
            turret_constants = self.robot_constants.shooter.turret
            # This feedforward voltage is added after all the scaled
            # calculations onboard the motor controller, so we need to account
            # for the gear reductions.
            feed_forward = (
                self._motion_magic_feed_forward
                * self._yaw_rate_signal.value
                * turret_constants.rotor_to_sensor_ratio
                * turret_constants.sensor_to_mechanism_ratio
                * self.DEGREES_TO_ROTATIONS
            )
            self.turret_motor.set_control(
                self._request.with_position(
                    self._turret_postion_degrees * self.DEGREES_TO_ROTATIONS
                ).with_feed_forward(feed_forward)
            )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._turret_postion_degrees = 0.0
        self._turret_velocity_degrees_per_second = 0.0

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._turret_postion_degrees = 0.0
        self._turret_velocity_degrees_per_second = 0.0

    def setPosition(self, pos_degrees: float) -> None:
        """Set the target position of the turret.

        Args:
            pos_degrees: The target position for the turret to move to, in
                degrees.
        """
        self._turret_postion_degrees = pos_degrees

    def setVelocity(self, vel_degrees_per_second) -> None:
        """Set the target speed of the turret.

        Args:
            vel_degrees_per_second: The target speed for the turret to rotate at,
                in degrees per second.
        """
        self._turret_velocity_degrees_per_second = vel_degrees_per_second

    def setControlType(self, use_velocity: bool) -> None:
        """Set the type of turret control to be used: position or velocity

        Args:
            use_velocity: The type of controler that should be used. True for
                velocity control, False for position control.
        """
        self._is_velocity_controlled = use_velocity

    def setMotionMagicFeedForward(self, feed_forward: float) -> None:
        """Set the motion magic feed forward constant.

        Args:
            feed_forward: The feed forward constant to use.
        """
        self._motion_magic_feed_forward = feed_forward

    def isControlTypeVelocity(self) -> bool:
        """Get the type of turret control being used used: position or velocity"""
        return self._is_velocity_controlled

    def zeroEncoder(self) -> None:
        """Zeroes the encoder at its current position."""
        self.turret_encoder.set_position(0.0)

    @magicbot.feedback
    def get_measured_angle_degrees(self) -> phoenix6.units.degree:
        return (
            self._encoder_position_signal.value
            * self.ROTATIONS_TO_DEGREES
            / self.robot_constants.shooter.turret.sensor_to_mechanism_ratio
        )

    @magicbot.feedback
    def get_target_angle_degrees(self) -> phoenix6.units.degree:
        return self._turret_postion_degrees

    @magicbot.feedback
    def get_supply_current(self) -> phoenix6.units.ampere:
        return self.turret_motor.get_supply_current().value

    @magicbot.feedback
    def get_stator_current(self) -> phoenix6.units.ampere:
        return self.turret_motor.get_stator_current().value


class TurretTuner:
    """Component for tuning the turret gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable turret target position.
    """

    robot_constants: constants.RobotConstants
    turret_motor: phoenix6.hardware.TalonFX
    turret_encoder: phoenix6.hardware.CANcoder
    turret: Turret
    hub_tracker: shooter.HubTracker

    # Gains for position control of the turret.
    position_k_s = magicbot.tunable(0.0)
    position_k_v = magicbot.tunable(0.0)
    position_k_a = magicbot.tunable(0.0)
    position_k_p = magicbot.tunable(0.0)
    position_k_i = magicbot.tunable(0.0)
    position_k_d = magicbot.tunable(0.0)

    # Gains for velocity control of the turret.
    velocity_k_s = magicbot.tunable(0.0)
    velocity_k_v = magicbot.tunable(0.0)
    velocity_k_a = magicbot.tunable(0.0)
    velocity_k_p = magicbot.tunable(0.0)
    velocity_k_i = magicbot.tunable(0.0)
    velocity_k_d = magicbot.tunable(0.0)

    # Limits for motion magic.
    mm_cruise_velocity = magicbot.tunable(0.0)
    mm_acceleration = magicbot.tunable(0.0)
    mm_jerk = magicbot.tunable(0.0)

    # Feedforward for motion magic.
    mm_feed_forward = magicbot.tunable(0.0)

    # The target position of the turret.
    target_position = magicbot.tunable(0.0)
    target_velocity = magicbot.tunable(0.0)
    use_velocity = magicbot.tunable(False)

    # Auto-track hub
    auto_track = magicbot.tunable(False)

    def setup(self) -> None:
        """Set up initial state for the turret tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        turret_constants: shooter.TurretConstants = (
            self.robot_constants.shooter.turret
        )

        self.position_k_s = turret_constants.position_k_s
        self.position_k_v = turret_constants.position_k_v
        self.position_k_a = turret_constants.position_k_a
        self.position_k_p = turret_constants.position_k_p
        self.position_k_i = turret_constants.position_k_i
        self.position_k_d = turret_constants.position_k_d

        self.velocity_k_s = turret_constants.velocity_k_s
        self.velocity_k_v = turret_constants.velocity_k_v
        self.velocity_k_a = turret_constants.velocity_k_a
        self.velocity_k_p = turret_constants.velocity_k_p
        self.velocity_k_i = turret_constants.velocity_k_i
        self.velocity_k_d = turret_constants.velocity_k_d

        self.mm_cruise_velocity = turret_constants.motion_magic_cruise_velocity
        self.mm_acceleration = turret_constants.motion_magic_acceleration
        self.mm_jerk = turret_constants.motion_magic_jerk

        self.mm_feed_forward = turret_constants.motion_magic_feed_forward

        self.last_position_k_s = self.position_k_s
        self.last_position_k_v = self.position_k_v
        self.last_position_k_a = self.position_k_a
        self.last_position_k_p = self.position_k_p
        self.last_position_k_i = self.position_k_i
        self.last_position_k_d = self.position_k_d

        self.last_velocity_k_s = self.velocity_k_s
        self.last_velocity_k_v = self.velocity_k_v
        self.last_velocity_k_a = self.velocity_k_a
        self.last_velocity_k_p = self.velocity_k_p
        self.last_velocity_k_i = self.velocity_k_i
        self.last_velocity_k_d = self.velocity_k_d

        self.last_mm_cruise_velocity = self.mm_cruise_velocity
        self.last_mm_acceleration = self.mm_acceleration
        self.last_mm_jerk = self.mm_jerk

        self.last_use_velocity = self.use_velocity

        self.logger.info("TurretTuner initialized")

    def execute(self) -> None:
        """Update the turret position and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.turret.setPosition(self.target_position)
        self.turret.setVelocity(self.target_velocity)
        self.turret.setControlType(self.use_velocity)
        self.turret.setMotionMagicFeedForward(self.mm_feed_forward)
        self.hub_tracker.trackPosition(self.auto_track)
        self.hub_tracker.trackSpeed(self.auto_track)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if not self.gainsChanged():
            return

        self.applyGains()

        self.last_use_velocity = self.use_velocity

        self.last_position_k_s = self.position_k_s
        self.last_position_k_v = self.position_k_v
        self.last_position_k_a = self.position_k_a
        self.last_position_k_p = self.position_k_p
        self.last_position_k_i = self.position_k_i
        self.last_position_k_d = self.position_k_d

        self.last_velocity_k_s = self.velocity_k_s
        self.last_velocity_k_v = self.velocity_k_v
        self.last_velocity_k_a = self.velocity_k_a
        self.last_velocity_k_p = self.velocity_k_p
        self.last_velocity_k_i = self.velocity_k_i
        self.last_velocity_k_d = self.velocity_k_d

        self.last_mm_cruise_velocity = self.mm_cruise_velocity
        self.last_mm_acceleration = self.mm_acceleration
        self.last_mm_jerk = self.mm_jerk

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.position_k_s != self.last_position_k_s
            or self.position_k_v != self.last_position_k_v
            or self.position_k_a != self.last_position_k_a
            or self.position_k_p != self.last_position_k_p
            or self.position_k_i != self.last_position_k_i
            or self.position_k_d != self.last_position_k_d
            or self.velocity_k_s != self.last_velocity_k_s
            or self.velocity_k_v != self.last_velocity_k_v
            or self.velocity_k_a != self.last_velocity_k_a
            or self.velocity_k_p != self.last_velocity_k_p
            or self.velocity_k_i != self.last_velocity_k_i
            or self.velocity_k_d != self.last_velocity_k_d
            or self.mm_cruise_velocity != self.last_mm_cruise_velocity
            or self.mm_acceleration != self.last_mm_acceleration
            or self.mm_jerk != self.last_mm_jerk
        )

    def applyGains(self) -> None:
        """Apply the current gains to the motor."""
        self.logger.info("Applying turret gains...")
        slot1_configs = (
            phoenix6.configs.Slot1Configs()
            .with_k_s(self.velocity_k_s)
            .with_k_v(self.velocity_k_v)
            .with_k_a(self.velocity_k_a)
            .with_k_p(self.velocity_k_p)
            .with_k_i(self.velocity_k_i)
            .with_k_d(self.velocity_k_d)
        )
        slot0_configs = (
            phoenix6.configs.Slot0Configs()
            .with_k_s(self.position_k_s)
            .with_k_v(self.position_k_v)
            .with_k_a(self.position_k_a)
            .with_k_p(self.position_k_p)
            .with_k_i(self.position_k_i)
            .with_k_d(self.position_k_d)
        )
        motion_magic_configs = (
            phoenix6.configs.MotionMagicConfigs()
            .with_motion_magic_cruise_velocity(self.mm_cruise_velocity)
            .with_motion_magic_acceleration(self.mm_acceleration)
            .with_motion_magic_jerk(self.mm_jerk)
        )
        result = self.turret_motor.configurator.apply(
            self.turret.turret_motor_configs.with_slot0(slot0_configs)
            .with_slot1(slot1_configs)
            .with_motion_magic(motion_magic_configs)
        )
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to turret motor")

    @magicbot.feedback
    def get_measured_dps(self) -> float:
        return self.turret_motor.get_velocity().value * 360
