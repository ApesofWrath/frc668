from dataclasses import dataclass

from phoenix6 import signals, units


@dataclass(frozen=True)
class HopperConstants:
    left_motor_can_id: int = 0
    left_motor_can_bus: str = ""
    left_motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    left_k_s: float = 0.0
    left_k_v: float = 0.0
    left_k_a: float = 0.0
    left_k_p: float = 0.0
    left_k_i: float = 0.0
    left_k_d: float = 0.0
    right_motor_can_id: int = 0
    right_motor_can_bus: str = ""
    right_motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    right_k_s: float = 0.0
    right_k_v: float = 0.0
    right_k_a: float = 0.0
    right_k_p: float = 0.0
    right_k_i: float = 0.0
    right_k_d: float = 0.0
    default_speed_rps: units.rotations_per_second = 30.0


@dataclass(frozen=True)
class IndexerConstants:
    back_motor_can_id: int = 0
    back_motor_can_bus: str = ""
    back_motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    back_k_s: float = 0.0
    back_k_v: float = 0.0
    back_k_a: float = 0.0
    back_k_p: float = 0.0
    back_k_i: float = 0.0
    back_k_d: float = 0.0
    front_motor_can_id: int = 0
    front_motor_can_bus: str = ""
    front_motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    front_k_s: float = 0.0
    front_k_v: float = 0.0
    front_k_a: float = 0.0
    front_k_p: float = 0.0
    front_k_i: float = 0.0
    front_k_d: float = 0.0
    default_speed_rps: units.rotations_per_second = 20.0


@dataclass(frozen=True)
class FlywheelConstants:
    encoder_can_id: int = 0
    encoder_can_bus: str = ""
    encoder_direction: signals.SensorDirectionValue = (
        signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    )
    motor_can_id: int = 0
    motor_can_bus: str = ""
    motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    k_s: float = 0.0
    k_v: float = 0.0
    k_a: float = 0.0
    k_p: float = 0.0
    k_i: float = 0.0
    k_d: float = 0.0
    default_speed_rps: units.rotations_per_second = 1.0


@dataclass(frozen=True)
class TurretConstants:
    sensor_to_mechanism_ratio: float = 10.0
    rotor_to_sensor_ratio: float = 4.167
    encoder_can_id: int = 0
    encoder_can_bus: str = ""
    encoder_direction: signals.SensorDirectionValue = (
        signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    )
    motor_can_id: int = 0
    motor_can_bus: str = ""
    motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    # The positive discontinuity point of the absolute encoder in rotations.
    # This determines the point at which the sensor wraps around, keeping the
    # absolute position (after offset) in the interval [x-1, x).
    absolute_sensor_discontinuity_point: units.rotation = 0.5
    # This offset is added to the reported position, allowing the application to
    # trim the zero position.
    magnet_offset: units.rotation = 0.0
    # Gains for position control.
    position_k_s: float = 0.0
    position_k_v: float = 0.0
    position_k_a: float = 0.0
    position_k_p: float = 0.0
    position_k_i: float = 0.0
    position_k_d: float = 0.0
    # Gains for velocity control.
    velocity_k_s: float = 0.0
    velocity_k_v: float = 0.0
    velocity_k_a: float = 0.0
    velocity_k_p: float = 0.0
    velocity_k_i: float = 0.0
    velocity_k_d: float = 0.0
    # Motion magic limits.
    motion_magic_cruise_velocity: units.rotations_per_second = 10.0
    motion_magic_acceleration: units.rotations_per_second_squared = 7.0
    motion_magic_jerk: units.rotations_per_second_cubed = 75.0
    # Feedforward for motion magic.
    motion_magic_feed_forward: units.voltage = 0.0
    # Limits for turret motion.
    min_angle: units.degree = -180.0
    max_angle: units.degree = 180.0


@dataclass(frozen=True)
class HoodConstants:
    sensor_to_mechanism_ratio: float = 19.0
    rotor_to_sensor_ratio: float = 8.0
    encoder_can_id: int = 0
    encoder_can_bus: str = ""
    encoder_direction: signals.SensorDirectionValue = (
        signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    )
    motor_can_id: int = 0
    motor_can_bus: str = ""
    motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    absolute_sensor_discontinuity_point: units.rotation = 0.8
    magnet_offset: units.rotation = 0.0
    k_s: float = 0.0
    k_v: float = 0.0
    k_a: float = 0.0
    k_g: float = 0.0
    k_p: float = 0.0
    k_i: float = 0.0
    k_d: float = 0.0
    min_angle_degrees: units.degree = 0.5
    max_angle_degrees: units.degree = 28.8
    # Motion magic limits.
    motion_magic_cruise_velocity: units.rotations_per_second = 10.0
    motion_magic_acceleration: units.rotations_per_second_squared = 5.0
    motion_magic_jerk: units.rotations_per_second_cubed = 100.0


@dataclass(frozen=True)
class ShooterConstants:
    hopper: HopperConstants = HopperConstants()
    indexer: IndexerConstants = IndexerConstants()
    flywheel: FlywheelConstants = FlywheelConstants()
    turret: TurretConstants = TurretConstants()
    hood: HoodConstants = HoodConstants()


SHOOTER_CONSTANTS: dict[str, ShooterConstants] = {
    # Alphabot
    "023AC96C": ShooterConstants(
        hopper=HopperConstants(
            left_motor_can_id=55,
            left_motor_can_bus="rio",
            left_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            left_k_s=0.5,
            left_k_v=0.123,
            left_k_p=0.5,
            right_motor_can_id=59,
            right_motor_can_bus="rio",
            right_motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            right_k_s=0.2,
            right_k_v=0.123,
            right_k_p=0.5,
        ),
        indexer=IndexerConstants(
            back_motor_can_id=18,
            back_motor_can_bus="Shooter",
            back_motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            back_k_s=0.4,
            back_k_v=0.12,
            back_k_p=0.25,
            front_motor_can_id=19,
            front_motor_can_bus="Shooter",
            front_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            front_k_s=0.35,
            front_k_v=0.105,
            front_k_p=0.2,
        ),
        flywheel=FlywheelConstants(
            encoder_can_id=60,
            encoder_can_bus="Shooter",
            encoder_direction=signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
            motor_can_id=15,
            motor_can_bus="Shooter",
            motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            k_s=0.2,
            k_v=0.13,
            k_p=0.25,
        ),
        turret=TurretConstants(
            encoder_can_id=16,
            encoder_can_bus="Shooter",
            encoder_direction=signals.SensorDirectionValue.CLOCKWISE_POSITIVE,
            motor_can_id=13,
            motor_can_bus="Shooter",
            motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            magnet_offset=-0.3892,
            position_k_p=50,
            position_k_d=0.1,
            velocity_k_s=0.24,
            velocity_k_v=1.5,
            velocity_k_p=1.0,
            min_angle=-90.0,
            max_angle=90.0,
        ),
        hood=HoodConstants(
            encoder_can_id=17,
            encoder_can_bus="Shooter",
            encoder_direction=signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
            motor_can_id=14,
            motor_can_bus="Shooter",
            motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            magnet_offset=-0.453,
            k_s=0.5,
            k_g=0.125,
            k_p=400.0,
        ),
    ),
    # Juno
    "0323CA4B": ShooterConstants(
        hopper=HopperConstants(
            left_motor_can_id=55,
            left_motor_can_bus="rio",
            left_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            left_k_s=0.1,
            left_k_v=0.16,
            left_k_p=0.5,
            right_motor_can_id=59,
            right_motor_can_bus="rio",
            right_motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            right_k_s=0.1,
            right_k_v=0.15,
            right_k_p=0.5,
        ),
        indexer=IndexerConstants(
            back_motor_can_id=19,
            back_motor_can_bus="rio",
            back_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            back_k_s=0.4,
            back_k_v=0.11,
            back_k_p=0.3,
            front_motor_can_id=18,
            front_motor_can_bus="rio",
            front_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            front_k_s=0.4,
            front_k_v=0.13,
            front_k_p=0.5,
        ),
        flywheel=FlywheelConstants(
            encoder_can_id=60,
            encoder_can_bus="rio",
            encoder_direction=signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
            motor_can_id=15,
            motor_can_bus="rio",
            motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
            k_s=0.4,
            k_v=0.12,
            k_p=1.0,
        ),
        turret=TurretConstants(
            rotor_to_sensor_ratio=3.42857,
            encoder_can_id=16,
            encoder_can_bus="rio",
            encoder_direction=signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
            motor_can_id=13,
            motor_can_bus="rio",
            motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            magnet_offset=0.197021,
            position_k_p=450,
            position_k_d=0.5,
        ),
        hood=HoodConstants(
            encoder_can_id=17,
            encoder_can_bus="rio",
            encoder_direction=signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
            motor_can_id=14,
            motor_can_bus="rio",
            motor_inverted=signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE,
            magnet_offset=0.1381,
            k_s=0.5,
            k_g=0.15,
            k_p=500.0,
        ),
    ),
}
