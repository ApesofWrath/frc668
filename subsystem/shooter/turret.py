import magicbot
import phoenix6

from phoenix6.controls import PositionVoltage, VelocityVoltage

from subsystem import shooter


class Turret:

    turret_motor: phoenix6.hardware.TalonFX
    turret_encoder: phoenix6.hardware.CANcoder

    def setup(self) -> None:
        """Set up initial state for the turret.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # Target position for the turret, in degrees. Counter clockwise is positive.
        self._turret_postion_degrees = 0.0

        # Target rotation speed for the turret, in degrees per second. Counter clockwise is positive.
        self._turret_velocity_degrees_per_second = 0.0
        self._is_velocity_controlled = False

        self.turret_configs = phoenix6.configs.TalonFXConfiguration()
        self.turret_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.FUSED_CANCODER
        )
        self.turret_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.TURRET_ENCODER_CAN_ID
        )
        self.turret_configs.feedback.sensor_to_mechanism_ratio = (
            shooter.constants.TURRET_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
        self.turret_configs.feedback.rotor_to_sensor_ratio = (
            shooter.constants.TURRET_ROTOR_TO_SENSOR_GEAR_RATIO
        )
        self.turret_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.CLOCKWISE_POSITIVE
        )
        self.turret_configs.slot0.k_p = shooter.constants.TURRET_K_P
        self.turret_configs.slot0.k_i = shooter.constants.TURRET_K_I
        self.turret_configs.slot0.k_d = shooter.constants.TURRET_K_D

        
        self.turret_configs.slot1.k_s = shooter.constants.TURRET_VEL_K_S
        self.turret_configs.slot1.k_v = shooter.constants.TURRET_VEL_K_V
        self.turret_configs.slot1.k_a = shooter.constants.TURRET_VEL_K_A
        self.turret_configs.slot1.k_p = shooter.constants.TURRET_VEL_K_P
        self.turret_configs.slot1.k_i = shooter.constants.TURRET_VEL_K_I
        self.turret_configs.slot1.k_d = shooter.constants.TURRET_VEL_K_D
        self.turret_motor.configurator.apply(self.turret_configs)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        if self._is_velocity_controlled:
            self.turret_motor.set_control(VelocityVoltage(self._turret_velocity_degrees_per_second/360).with_slot(1))
        else:
            self.turret_motor.set_control(PositionVoltage(self._turret_postion_degrees/360).with_slot(0))

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
            pos_degrees: The target position for the turret to move to, in degrees.
        """
        self._turret_postion_degrees = pos_degrees

    def setVelocity(self, vel_degrees: float) -> None:
        """Set the target speed of the turret.
        
        Args:
            vel_degrees: The target speed for the turret to rotate at, in degrees per second.
        """
        self._turret_velocity_degrees_per_second = vel_degrees

    def setControlType(self, use_velocity: bool) -> None:
        """Set the type of turret control to be used: position or velocity

        Args:
            use_velocity: The type of controler that should be used. True for velocity control, False for position control. 
        """
        self._is_velocity_controlled = use_velocity

    def isControlTypeVelocity(self) -> bool:
        """Get the type of turret control being used used: position or velocity"""
        return self._is_velocity_controlled

    def zeroEncoder(self) -> None:
        """Zeroes the encoder at its current position."""
        self.turret_encoder.set_position(0.0)

    @magicbot.feedback
    def get_turret_angle(self) -> float:
        return (
            self.turret_encoder.get_position().value
            * 360.0
            / shooter.constants.TURRET_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
    
class TurretTuner:
    """Component for tuning the turret gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable turret target position.
    """

    turret_motor: phoenix6.hardware.TalonFX
    turret_encoder: phoenix6.hardware.CANcoder
    turret: Turret

    # Gains for position control of the turret.
    k_p = magicbot.tunable(shooter.constants.TURRET_K_P)
    k_i = magicbot.tunable(shooter.constants.TURRET_K_I)
    k_d = magicbot.tunable(shooter.constants.TURRET_K_D)
    
    # Gains for velocity control of the turret.
    vel_k_s = magicbot.tunable(shooter.constants.TURRET_VEL_K_S)
    vel_k_v = magicbot.tunable(shooter.constants.TURRET_VEL_K_V)
    vel_k_a = magicbot.tunable(shooter.constants.TURRET_VEL_K_A)
    vel_k_p = magicbot.tunable(shooter.constants.TURRET_VEL_K_P)
    vel_k_i = magicbot.tunable(shooter.constants.TURRET_VEL_K_I)
    vel_k_d = magicbot.tunable(shooter.constants.TURRET_VEL_K_D)

    # The target position of the turret.
    target_position = magicbot.tunable(0.0)
    target_velocity = magicbot.tunable(0.0)
    use_velocity = magicbot.tunable(False)

    def setup(self) -> None:
        """Set up initial state for the turret tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self.last_k_p = shooter.constants.TURRET_K_P
        self.last_k_i = shooter.constants.TURRET_K_I
        self.last_k_d = shooter.constants.TURRET_K_D
        
        self.last_use_velocity = self.use_velocity

        self.last_vel_k_s = shooter.constants.TURRET_VEL_K_S
        self.last_vel_k_v = shooter.constants.TURRET_VEL_K_V
        self.last_vel_k_a = shooter.constants.TURRET_VEL_K_A
        self.last_vel_k_p = shooter.constants.TURRET_VEL_K_P
        self.last_vel_k_i = shooter.constants.TURRET_VEL_K_I
        self.last_vel_k_d = shooter.constants.TURRET_VEL_K_D

        self.logger.info("Turret initialized")

    def execute(self) -> None:
        """Update the turret position and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.turret.setPosition(self.target_position)
        self.turret.setVelocity(self.target_velocity)
        self.turret.setControlType(self.use_velocity)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if not self.gainsChanged():
            return

        self.logger.info("Gains changed!")
        self.applyGains()

        self.last_use_velocity = self.use_velocity

        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d

        self.last_vel_k_s = self.vel_k_s
        self.last_vel_k_v = self.vel_k_v
        self.last_vel_k_a = self.vel_k_a
        self.last_vel_k_p = self.vel_k_p
        self.last_vel_k_i = self.vel_k_i
        self.last_vel_k_d = self.vel_k_d

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.k_p != self.last_k_p
            or self.k_i != self.last_k_i
            or self.k_d != self.last_k_d
            or self.vel_k_s != self.last_vel_k_s
            or self.vel_k_v != self.last_vel_k_v
            or self.vel_k_a != self.last_vel_k_a
            or self.vel_k_p != self.last_vel_k_p
            or self.vel_k_i != self.last_vel_k_i
            or self.vel_k_d != self.last_vel_k_d
        )

    def applyGains(self) -> None:
        """Apply the current gains to the motor."""
        self.logger.info("applying gains")
        slot1_configs = (
            phoenix6.configs.config_groups.Slot1Configs()
            .with_k_s(self.vel_k_s)
            .with_k_v(self.vel_k_v)
            .with_k_a(self.vel_k_a)
            .with_k_p(self.vel_k_p)
            .with_k_i(self.vel_k_i)
            .with_k_d(self.vel_k_d)
        )
        slot0_configs = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )
        result = self.turret_motor.configurator.apply(self.turret.turret_configs.with_slot0(slot0_configs).with_slot1(slot1_configs))
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to turret motor")

    @magicbot.feedback
    def get_motor_voltage(self) -> phoenix6.units.volt:
        return self.turret_motor.get_motor_voltage().value

    @magicbot.feedback
    def get_motor_supply_current(self) -> phoenix6.units.ampere:
        return self.turret_motor.get_supply_current().value

    @magicbot.feedback
    def get_motor_stator_current(self) -> phoenix6.units.ampere:
        return self.turret_motor.get_stator_current().value

    @magicbot.feedback
    def get_position(self) -> float:
        return self.turret_encoder.get_position().value
    
    @magicbot.feedback
    def get_measured_dps(self) -> float:
        return self.turret_motor.get_velocity().value * 360