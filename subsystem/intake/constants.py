from dataclasses import dataclass

from phoenix6 import signals


@dataclass(frozen=True)
class IntakeConstants:
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


INTAKE_CONSTANTS: dict[str, IntakeConstants] = {
    # Alphabot
    "023AC96C": IntakeConstants(
        motor_can_id=41,
        motor_can_bus="Shooter",
        motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
        # TODO: Tune.
        k_s=0.0,
        k_v=0.0,
        k_a=0.0,
        k_p=0.0,
        k_i=0.0,
        k_d=0.0,
    ),
    # Juno
    "0323CA4B": IntakeConstants(
        motor_can_id=41,
        motor_can_bus="rio",
        motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
        k_s=0.25,
        k_v=0.12,
        k_a=0.0,
        k_p=0.2,
        k_i=0.0,
        k_d=0.0,
    ),
}
