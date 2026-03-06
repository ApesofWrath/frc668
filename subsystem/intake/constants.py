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
    deploy_encoder_can_id: int = 0
    sensor_to_mechanism_ratio: int = 0
    rotor_to_sensor_ratio: int = 0
    motor_inverted: bool = False
    position_k_s: int = 0
    position_k_v: int = 0
    position_k_a: int = 0
    position_k_p: int = 0
    position_k_i: int = 0
    position_k_d: int = 0


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
        deploy_encoder_can_id=61
        sensor_to_mechanism_ratio=
        rotor_to_sensor_ratio=
        motor_inverted=False
        position_k_s
        position_k_v
        position_k_a
        position_k_p
        position_k_i
        position_k_d
    )
}
