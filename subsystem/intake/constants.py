from dataclasses import dataclass

from phoenix6 import signals, units


@dataclass(frozen=True)
class IntakeConstants:
    motor_can_id: int = 0
    motor_can_bus: str = ""
    motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )
    motor_inverted: signals.InvertedValue = (signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE)
    deploy_motor_can_id: int = 0
    deploy_motor_can_bus: str = ""
    deploy_encoder_can_id: int = 0
    deploy_encoder_can_bus: str = ""
    sensor_to_mechanism_ratio: int = 0
    rotor_to_sensor_ratio: int = 0
    encoder_direction: signals.SensorDirectionValue = (
        signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    )
    absolute_sensor_discontinuity_point: units.rotation = 0.0
    position_k_s: float = 0
    position_k_v: float = 0
    position_k_a: float = 0
    position_k_p: float = 0
    position_k_i: float = 0
    position_k_d: float = 0
    motion_magic_cruise_velocity: float = 0.0
    motion_magic_acceleration: float = 0.0
    motion_magic_jerk: float = 0.0
    motion_magic_feed_forward: float = 0.0



INTAKE_CONSTANTS: dict[str, IntakeConstants] = {
    # Alphabot
    "023AC96C": IntakeConstants(
        motor_can_id=41,
        motor_can_bus="Shooter",
        motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
    ),
    # Juno
    "0323CA4B": IntakeConstants(
        motor_can_id=41,
        motor_can_bus="rio",
        motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
        deploy_motor_can_id=50,
        deploy_encoder_can_id=61,
        sensor_to_mechanism_ratio= 9/30,
        rotor_to_sensor_ratio= 25,
        position_k_s=0 ,
        position_k_v=0 ,
        position_k_a=0 ,
        position_k_p=0 ,
        position_k_i=0 ,
        position_k_d=0 ,
        deploy_motor_inverted = ,
        deploy_encoder_inverted = 
    )

    )}
