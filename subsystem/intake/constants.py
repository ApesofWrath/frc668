from dataclasses import dataclass

from phoenix6 import signals, units


@dataclass(frozen=True)
class IntakeConstants:
    roller_motor_can_id: int = 0
    roller_motor_can_bus: str = ""
    roller_motor_inverted: signals.InvertedValue = signals.InvertedValue.CLOCKWISE_POSITIVE
    deploy_motor_can_id: int = 0
    deploy_motor_can_bus: str = ""
    deploy_motor_inverted: signals.InvertedValue = signals.InvertedValue.CLOCKWISE_POSITIVE
    deploy_encoder_can_id: int = 0
    deploy_encoder_can_bus: str = ""
    deploy_encoder_direction: signals.SensorDirectionValue = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    deploy_sensor_to_mechanism_ratio: int = 0
    deploy_rotor_to_sensor_ratio: int = 0
    deploy_absolute_sensor_discontinuity_point: units.rotation = 0.0



INTAKE_CONSTANTS: dict[str, IntakeConstants] = {
    # Alphabot
    "023AC96C": IntakeConstants(
        roller_motor_can_id=41,
        roller_motor_can_bus="Shooter",
        roller_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
    ),
    # Juno
    "0323CA4B": IntakeConstants(
        roller_motor_can_id=41,
        roller_motor_can_bus="rio",
        roller_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
        deploy_motor_can_id=50,
        deploy_motor_can_bus="rio",
        deploy_motor_inverted=signals.InvertedValue.CLOCKWISE_POSITIVE,
        deploy_encoder_can_id=61,
        deploy_encoder_can_bus="rio",
        deploy_encoder_direction=signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE,
        deploy_sensor_to_mechanism_ratio= 3.3333333,
        deploy_rotor_to_sensor_ratio= 25.0,
    ),
}
