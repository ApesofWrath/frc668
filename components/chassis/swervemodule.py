# Credit to https://github.com/olentangyfrc/2025-Reefscape-Code 

import math

from wpimath.controller import SimpleMotorFeedforwardMeters, ProfiledPIDController
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
import wpimath.trajectory

from phoenix6 import hardware, configs, controls, BaseStatusSignal, signals
from wpilib import RobotBase

import constants

wheel_radius = constants.WHEEL_RADIUS
max_rotation_speed = constants.MAX_SINGLE_SWERVE_ROTATION_SPEED
max_rotation_acceleration = constants.MAX_SINGLE_SWERVE_ROTATION_ACCELERATION

class SwerveModule:
    def __init__(
        self,
        drive_motor_id: int,
        turning_motor_id: int,
        turning_encoder_id: int,
        offset: float,
        name: str,
    ) -> None:
        """
        Initializes the given swerve module

        Params:
            drive_motor_id (int): CAN ID of the drive motor for the given module
            turning_motor_id (int): CAN ID of the turning motor for the given module
            turning_encoder_id (int): CAN ID of the absolute encoder attached to the given module
            offset (float): The turning offset in degrees of the given module
            name (str): Name of the given module
        """
        self._offset = offset

        drive_motor_configs = configs.TalonFXConfiguration()
        drive_motor_configs.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
        drive_motor_configs.current_limits.stator_current_limit_enable = True
        drive_motor_configs.current_limits.stator_current_limit = 50
        drive_motor_configs.future_proof_configs = True
        if name in ["Front Right", "Front Left"]:
            drive_motor_configs.motor_output.inverted = (
                signals.InvertedValue.CLOCKWISE_POSITIVE
            )

        self.drive_motor = hardware.talon_fx.TalonFX(drive_motor_id, "rio")
        self.drive_motor.clear_sticky_faults()
        self.drive_motor.configurator.apply(drive_motor_configs)

        self.turning_motor_configs = configs.TalonFXConfiguration()
        self.turning_motor_configs.motor_output.neutral_mode = (
            signals.NeutralModeValue.BRAKE
        )
        self.turning_motor_configs.motor_output.inverted = (
            signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )
        self.turning_motor_configs.future_proof_configs = True

        self.turning_motor = hardware.TalonFX(turning_motor_id, "rio")
        self.turning_motor.clear_sticky_faults()
        self.turning_motor.configurator.apply(self.turning_motor_configs)

        self.turning_encoder = hardware.cancoder.CANcoder(turning_encoder_id, "rio")

        self.turning_motor.set_control(controls.VoltageOut(0))

        self.name = name

        # Gains are for example purposes only - must be determined for your own robot!
        #TODO: tune! 
        self.drive_feed_forward = SimpleMotorFeedforwardMeters(
            0.23727, 2.59628, 0                    
        )  # How fast motor turns with respect to how fast it is commanded to drive

        self.turning_PID_controller = ProfiledPIDController(
            constants.TURNING_P,
            constants.TURNING_I,
            constants.TURNING_D,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                max_rotation_speed, max_rotation_acceleration
            ),
        )

        self.turning_PID_controller.enableContinuousInput(-math.pi, math.pi)
        self.turning_PID_controller.setTolerance(math.pi / 180)

        if not RobotBase.isReal():
            self.sim_angle = Rotation2d()
            self.sim_position = 0.0
            self.sim_velocity = 0.0

    @property
    def offset(self) -> float:
        """
        Offset of the CANcoder for steering rotation in degrees
        """
        return self._offset

    @offset.setter
    def offset(self, offset_value) -> None:
        """
        Sets the local offset variable
        """
        if isinstance(offset_value, (int, float)):
            self._offset = offset_value
        else:
            raise ValueError("Can't input that value for offset")

    def getEncoderAngle(self) -> float:
        """
        Gets the steering encoder angle in radians

        Returns:
            float: The turning angle of the module in radians
        """
        if not RobotBase.isReal():
            return self.sim_angle.radians()
        angle = BaseStatusSignal.get_latency_compensated_value(
            self.turning_encoder.get_absolute_position(),
            self.turning_encoder.get_velocity(),
        )
        return angle * math.tau - (self.offset * math.tau) #to convert to radians

    def getEncoderAngleDeg(self) -> float:
        """
        Gets the steering encoder angle in degrees

        Returns:
            float: The steering encoder angle in degrees
        """
        return (self.getEncoderAngle() * 180 / math.pi) % 360

    def getEncoderAngleAbs(self) -> float:
        return (self.turning_encoder.get_absolute_position().value * 360) % 360

    def getDriveMotorVelocity(self) -> float:
        """
        Gets the rotational velocity of the swerve module on the steering axis in radians / sec

        Returns:
            float: Rotational velocity in radians per second
        """
        return (
            self.drive_motor.get_velocity().value
            * constants.GEAR_RATIO
            * constants.WHEEL_RADIUS
            * math.tau
        )

    def getDriveMotorPosition(self) -> float:
        """
        Gets the total translational distance of the drive motor in meters

        Returns:
            float: Total distance traveled in meters
        """
        if not RobotBase.isReal():
            return self.sim_position
        position = BaseStatusSignal.get_latency_compensated_value(
            self.drive_motor.get_position(), self.drive_motor.get_velocity()
        )
        return position * constants.WHEEL_RADIUS * math.tau * constants.GEAR_RATIO

    def resetDriveMotorPosition(self) -> None:
        """
        Resets the drive motor encoder to 0 at the start of the match
        """
        self.drive_motor.set_position(0)

    def getSwervePosition(self) -> SwerveModulePosition:
        """
        Gets the current position of the given swerve module for both the drive and steer positions

        Returns:
            SwerveModulePosition: the current position and angle of the swerve module
        """
        return SwerveModulePosition(
            self.getDriveMotorPosition(), Rotation2d(self.getEncoderAngle())
        )

    def setModuleVoltage(self, drive_voltage: float, turn_voltage: float) -> None:
        """
        Sets the voltage for the drive and steer motors

        Params:
            drive_voltage (float): Voltage fed to the drive motor
            turn_voltage (float): Voltage fed to the turn motor
        """

        self.drive_motor.set_control(controls.VoltageOut(drive_voltage))
        self.turning_motor.set_control(controls.VoltageOut(turn_voltage))

    def getModuleVoltage(self) -> list[float]:
        """
        Gets the current voltage given to each module

        Returns:
            list[float]: A list containing the voltage fed to the drive motor then the turn motor
        """
        return [
            self.drive_motor.get_motor_voltage().value,
            self.turning_motor.get_motor_voltage().value,
        ]

    def stopSwerveModule(self) -> None:
        """
        Stops all movement for this swerve module. Only supposed to be used in emergency / urgent situations
        """
        self.setModuleVoltage(0, 0)
        self.setDesiredState(
            SwerveModuleState(0, Rotation2d(self.getEncoderAngle()))
        )

    def resetFeedforward(self) -> None:
        """
        Resets the drive feedfoward on module to only
        having a Kv of 1 (basically voltage = inputted velocity). Only use when tuning feedfowards.
        """
        self.drive_feed_forward.setKs(0)
        self.drive_feed_forward.setKv(1)
        self.drive_feed_forward.setKa(0)

    def setDesiredState(self, desired_state: SwerveModuleState) -> None:
        """
        Given the desired state, this method runs the calculations and sets the voltages necessary
        to achieve that state.

        Params:
            desired_state (SwerveModuleState): The target state for the given swerve module to reach
        """

        encoder_rotation = Rotation2d(self.getEncoderAngle())

        # Optimize the reference state to avoid spinning further than 90 degrees
        desired_state.optimize(encoder_rotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.

        desired_state.cosineScale(encoder_rotation)

        drive_output = self.drive_feed_forward.calculate(desired_state.speed)

        turn_output = self.turning_PID_controller.calculate(
            self.getEncoderAngle(), desired_state.angle.radians()
        )

        self.setModuleVoltage(drive_output, turn_output)

        if not RobotBase.isReal():
            self.sim_velocity = desired_state.speed
            self.sim_position += desired_state.speed * 0.02
            self.sim_angle = desired_state.angle
