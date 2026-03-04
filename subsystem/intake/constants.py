from dataclasses import dataclass

from phoenix6 import signals


@dataclass(frozen=True)
class IntakeConstants:
    motor_can_id: int = 0
    motor_can_bus: str = ""
    motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )


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
