import magicbot
import phoenix6

from phoenix6.controls import PositionVoltage

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

        turret_configs = phoenix6.configs.TalonFXConfiguration()
        turret_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.FUSED_CANCODER
        )
        turret_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.TURRET_ENCODER_CAN_ID
        )
        turret_configs.feedback.sensor_to_mechanism_ratio = (
            shooter.constants.TURRET_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
        turret_configs.feedback.rotor_to_sensor_ratio = (
            shooter.constants.TURRET_ROTOR_TO_SENSOR_GEAR_RATIO
        )
        turret_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.CLOCKWISE_POSITIVE
        )
        turret_configs.slot0.k_p = shooter.constants.TURRET_K_P
        turret_configs.slot0.k_i = shooter.constants.TURRET_K_I
        turret_configs.slot0.k_d = shooter.constants.TURRET_K_D
        self.turret_motor.configurator.apply(turret_configs)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.turret_motor.set_control(
            PositionVoltage(self._turret_postion_degrees / 360)
        )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._turret_postion_degrees = 0.0

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._turret_postion_degrees = 0.0

    def setPosition(self, pos_degrees: float) -> None:
        """Set the target position of the turret.

        Args:
            pos_degrees: The target position for the turret to move to, in degrees.
        """
        self._turret_postion_degrees = pos_degrees

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

    # The target position of the turret.
    target_position = magicbot.tunable(0.0)

    def setup(self) -> None:
        """Set up initial state for the turret tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self.last_k_p = shooter.constants.TURRET_K_P
        self.last_k_i = shooter.constants.TURRET_K_I
        self.last_k_d = shooter.constants.TURRET_K_D

    def execute(self) -> None:
        """Update the turret position and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.turret.setPosition(self.target_position)

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if not self.gainsChanged():
            return

        self.applyGains()

        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.k_p != self.last_k_p
            or self.k_i != self.last_k_i
            or self.k_d != self.last_k_d
        )

    def applyGains(self) -> None:
        """Apply the current gains to the motor."""
        slot0_configs = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )
        result = self.turret_motor.configurator.apply(slot0_configs)
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
