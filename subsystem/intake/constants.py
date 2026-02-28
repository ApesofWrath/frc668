from dataclasses import dataclass

from phoenix6 import signals


@dataclass(frozen=True)
class IntakeConstants:
    motor_can_id: int = 0
    motor_inverted: signals.InvertedValue = (
        signals.InvertedValue.COUNTER_CLOCKWISE_POSITIVE
    )


INTAKE_CONSTANTS: dict[str, IntakeConstants] = {
    "023AC96C": IntakeConstants(
        motor_can_id=41,
        motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
    )
}
