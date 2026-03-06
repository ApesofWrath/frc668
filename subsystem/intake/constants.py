from dataclasses import dataclass

from phoenix6 import signals


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
    ),
}

DEPLOY_INTAKE_CONSTANTS: dict[str, IntakeConstants] = {
    "0323CA4B": IntakeConstants(
        motor_can_id=41, 
        motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
        deploy_motor_can_id=50,
        deploy_encoder_can_id=61,
        sensor_to_mechanism_ratio= ,
        rotor_to_sensor_ratio= ,
        position_k_s= ,
        position_k_v= ,
        position_k_a= ,
        position_k_p= ,
        position_k_i= ,
        position_k_d= ,
        motion_magic_cruise_velocity= ,
        motion_magic_acceleration= ,
        motion_magic_jerk= ,
        motion_magic_feed_forward=
    )
}
