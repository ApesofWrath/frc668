import magicbot
import phoenix6
import wpilib

from subsystem import shooter
from subsystem.shooter.constants import HOOD_MAX_ANGLE, HOOD_MIN_ANGLE


class Hood:
    """Hood component

    This class controls the angle of the hood.
    """

    hood_motor: phoenix6.hardware.TalonFX
    hood_encoder: phoenix6.hardware.CANcoder

    def setup(self) -> None:
        """Set up initial state for the hood.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self._hood_speed = 0.0
        self._hood_target_position = 0.0  # self.get_hood_angle()

        self.is_manual = False
        # self.homing = False

        hood_configs = phoenix6.configs.TalonFXConfiguration()
        hood_configs.feedback.feedback_sensor_source = (
            phoenix6.signals.spn_enums.FeedbackSensorSourceValue.REMOTE_CANCODER
        )
        hood_configs.feedback.feedback_remote_sensor_id = (
            shooter.constants.HOOD_ENCODER_CAN_ID
        )
        hood_configs.feedback.sensor_to_mechanism_ratio = (
            shooter.constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        )
        hood_configs.feedback.rotor_to_sensor_ratio = (
            shooter.constants.HOOD_ROTOR_TO_SENSOR_GEAR_RATIO
        )
        hood_configs.motor_output.inverted = (
            phoenix6.signals.spn_enums.InvertedValue.COUNTER_CLOCKWISE_POSITIVE  # CLOCKWISE_POSITIVE
        )
        hood_configs.slot0.k_s = shooter.constants.HOOD_K_S
        hood_configs.slot0.k_v = shooter.constants.HOOD_K_V
        hood_configs.slot0.k_a = shooter.constants.HOOD_K_A
        hood_configs.slot0.k_p = shooter.constants.HOOD_K_P
        hood_configs.slot0.k_i = shooter.constants.HOOD_K_I
        hood_configs.slot0.k_d = shooter.constants.HOOD_K_D
        hood_configs.slot0.k_g = shooter.constants.HOOD_K_G

        # TODO: Configure soft limits.
        self.hood_motor.configurator.apply(hood_configs)

        encoder_configs = phoenix6.configs.CANcoderConfiguration()
        encoder_configs.magnet_sensor.sensor_direction = (
            phoenix6.signals.spn_enums.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE  # CLOCKWISE_POSITIVE
        )
        self.hood_encoder.configurator.apply(encoder_configs)

        self._request = phoenix6.controls.PositionVoltage(
            self._hood_target_position / 360.0
        ).with_slot(0)

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """

        self._hood_target_position = max(
            HOOD_MIN_ANGLE, min(HOOD_MAX_ANGLE, self._hood_target_position)
        )

        if self.is_manual:
            self.hood_motor.set(self._hood_speed)
        else:
            self.hood_motor.set_control(
                self._request.with_position(self._hood_target_position / 360.0)
            )

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._hood_speed = 0.0
        self._hood_target_position = self.get_hood_angle()

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._hood_speed = 0.0
        self._hood_target_position = self.get_hood_angle()

    def setSpeed(self, speed: float) -> None:
        """Set the speed of the hood."""
        self._hood_speed = speed

    def setPosition(self, target_position: float) -> None:
        """Set the position (in degrees) of the hood"""
        self._hood_target_position = max(
            HOOD_MIN_ANGLE, min(HOOD_MAX_ANGLE, target_position)
        )

    def zeroEncoder(self) -> None:
        """Zeroes the encoder at its current position."""
        self._hood_target_position = 0.0
        self.hood_encoder.set_position(0.0)

    @magicbot.feedback
    def get_hood_angle(self) -> float:
        return (
            self.hood_encoder.get_position().value
            * 360.0
            / shooter.constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        )

    @magicbot.feedback
    def get_motor_stator_current(self) -> float:
        return self.hood_motor.get_stator_current().value

    # def homing_routine(self) -> None:
    #     self.hood_motor.set(-0.1)
    #     if self.hood_motor.get_stator_current.value >= 8.5:
    #         self.zeroEncoder()
    #         self.hood_motor.set(0.0)
    #         self.homing = False


class HoodTuner:
    """Component for tuning the hood gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable hood target angle position.
    """

    hood_motor: phoenix6.hardware.TalonFX
    hood_encoder: phoenix6.hardware.CANcoder
    hood: Hood

    # Gains for position control of the hood.
    k_s = magicbot.tunable(shooter.constants.HOOD_K_S)
    k_v = magicbot.tunable(shooter.constants.HOOD_K_V)
    k_a = magicbot.tunable(shooter.constants.HOOD_K_A)
    k_p = magicbot.tunable(shooter.constants.HOOD_K_P)
    k_i = magicbot.tunable(shooter.constants.HOOD_K_I)
    k_d = magicbot.tunable(shooter.constants.HOOD_K_D)
    k_g = magicbot.tunable(shooter.constants.HOOD_K_G)

    # The target angle of the hood, in degrees.
    target_angle = magicbot.tunable(20.0)
    homing = magicbot.tunable(False)

    def setup(self) -> None:
        """Set up initial state for the hood tuner.
        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self.last_k_s = shooter.constants.HOOD_K_S
        self.last_k_v = shooter.constants.HOOD_K_V
        self.last_k_a = shooter.constants.HOOD_K_A
        self.last_k_p = shooter.constants.HOOD_K_P
        self.last_k_i = shooter.constants.HOOD_K_I
        self.last_k_d = shooter.constants.HOOD_K_D
        self.last_k_g = shooter.constants.HOOD_K_G

    def execute(self) -> None:
        """Update the hood speed and gains (if they changed).

        This method is called at the end of the control loop.
        """
        self.hood.setPosition(self.target_angle)
        if self.homing == True:
            self.hood.homing_routine()

        # We only want to reapply the gains if they changed. The TalonFX motor
        # doesn't like being reconfigured constantly.
        if not self.gainsChanged():
            return

        self.applyGains()

        self.last_k_s = self.k_s
        self.last_k_v = self.k_v
        self.last_k_a = self.k_a
        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d
        self.last_k_g = self.k_g

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.k_s != self.last_k_s
            or self.k_v != self.last_k_v
            or self.k_a != self.last_k_a
            or self.k_p != self.last_k_p
            or self.k_i != self.last_k_i
            or self.k_d != self.last_k_d
            or self.k_g != self.last_k_g
        )

    def applyGains(self) -> None:
        """Apply the current gains to the motor."""
        slot0_configs = (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.k_s)
            .with_k_v(self.k_v)
            .with_k_a(self.k_a)
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
            .with_k_g(self.k_g)
        )
        result = self.hood_motor.configurator.apply(slot0_configs)
        if not result.is_ok():
            self.logger.error("Failed to apply new gains to hood motor")

    @magicbot.feedback
    def get_motor_voltage(self) -> phoenix6.units.volt:
        return self.hood_motor.get_motor_voltage().value

    @magicbot.feedback
    def get_motor_supply_current(self) -> phoenix6.units.ampere:
        return self.hood_motor.get_supply_current().value

    @magicbot.feedback
    def get_motor_stator_current(self) -> phoenix6.units.ampere:
        return self.hood_motor.get_stator_current().value

    @magicbot.feedback
    def get_motor_torque_current(self) -> phoenix6.units.ampere:
        return self.hood_motor.get_torque_current().value

    @magicbot.feedback
    def get_motor_stall_current(self) -> phoenix6.units.ampere:
        return self.hood_motor.get_motor_stall_current().value

    @magicbot.feedback
    def get_hood_angle(self) -> float:
        return (
            self.hood_encoder.get_position().value
            * 360.0
            / shooter.constants.HOOD_SENSOR_TO_MECHANISM_GEAR_RATIO
        )

    @magicbot.feedback
    def get_encoder_position(self) -> float:
        return self.hood_encoder.get_position().value


class Homing(magicbot.StateMachine):
    hood_motor: phoenix6.hardware.TalonFX
    hood_encoder: phoenix6.hardware.CANcoder
    hood: Hood

    def __init__(self):
        self._first_spike = True
        self._timer = wpilib.Timer()
        self.zeroed = False

    def homing_routine(self) -> None:
        self.engage()
        self.logger.info(self.current_state)

    @magicbot.state(first=True, must_finish=True)
    def homing(self, state_tm) -> None:
        self.hood_motor.set(-0.1)
        if self.hood_motor.get_stator_current().value >= 8.5:
            if self._first_spike:
                self._first_spike = False
                self._timer.start()
            else:
                if self._timer.hasElapsed(0.1):
                    self._first_spike = True
                    self._timer.reset()
                    self.next_state("zero")
        elif state_tm >= 10.0:
            self.next_state("timeout")
        else:
            # self.hood_motor.set(-0.1)
            self.logger.info("Homing incomplete")

    @magicbot.state(must_finish=True)
    def zero(self) -> None:
        self.zeroed = True
        self.logger.info("Entered zeroing state.")
        self.hood_motor.set(0.0)
        # self.hood_motor.setNeutralMode(0)  # 0 is to coast, 1 is to brake TODO: check if erroneous 
        self.hood.zeroEncoder()
        # TODO: exit state 

    @magicbot.state()
    def timeout(self) -> None:
        self.hood_motor.set(0.0)
        self.logger.warning("Timeout, homing routine incomplete!")

    @magicbot.feedback
    def zeroed_success(self) -> None:
        return self.zeroed
