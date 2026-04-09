import magicbot
import phoenix6
import wpilib

import constants
from common import datalog
from subsystem import intake


class IntakeDeployer(magicbot.StateMachine):
    """Component that deploys the intake

    This class deploys our intake using a simple state machine.
    """

    robot_constants: constants.RobotConstants
    intake_deploy_motor: phoenix6.hardware.TalonFX
    intake_deploy_encoder: phoenix6.hardware.CANcoder
    intake: intake.Intake
    data_logger: datalog.DataLogger

    def __init__(self):
        self._deployed = False

    def setup(self) -> None:
        intake_constants = self.robot_constants.intake
        self.intake_deploy_encoder.configurator.apply(
            phoenix6.configs.CANcoderConfiguration().with_magnet_sensor(
                phoenix6.configs.MagnetSensorConfigs().with_sensor_direction(
                    intake_constants.deploy_encoder_direction
                )
            )
        )
        self.intake_deploy_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration()
            .with_feedback(
                phoenix6.configs.FeedbackConfigs()
                .with_feedback_sensor_source(
                    phoenix6.signals.FeedbackSensorSourceValue.FUSED_CANCODER
                )
                .with_feedback_remote_sensor_id(
                    intake_constants.deploy_encoder_can_id
                )
                .with_sensor_to_mechanism_ratio(
                    intake_constants.deploy_sensor_to_mechanism_ratio
                )
                .with_rotor_to_sensor_ratio(
                    intake_constants.deploy_rotor_to_sensor_ratio
                )
            )
            .with_motor_output(
                phoenix6.configs.MotorOutputConfigs()
                .with_inverted(intake_constants.deploy_motor_inverted)
                .with_neutral_mode(phoenix6.signals.NeutralModeValue.COAST)
            )
            .with_current_limits(
                phoenix6.configs.CurrentLimitsConfigs()
                .with_supply_current_limit(
                    self.robot_constants.intake.deploy_motor_supply_current_limit
                )
                .with_supply_current_limit_enable(True)
            )
        )

        # Assume the intake has been reset to its vertical position, and set the
        # mechanism position to zero.
        self.intake_deploy_encoder.set_position(0.0)

        self._log_timer = wpilib.Timer()
        self._log_timer.start()

    def deploy(self) -> None:
        """Deploy the intake.

        Call this each control loop when the robot is enabled.
        """
        if not self._deployed:
            self.engage()

        self._logData()

    def hasDeployed(self) -> bool:
        return self._deployed

    @magicbot.state(first=True)
    def deploying(self, state_tm) -> None:
        # Based on testing on the robot, deploying the intake corresponds to
        # roughly one full rotation on the encoder. Apply a motor input most of
        # the way there, but stop a bit early so that we don't push against the
        # bumper.
        if self.intake_deploy_encoder.get_position().value >= 0.85:
            self.next_state("deployed")
        elif state_tm >= 10.0:
            self.next_state("timed_out")

        self.intake.setActive(True)
        self.intake_deploy_motor.set(0.25)

    @magicbot.state
    def deployed(self):
        self.logger.info("Success: Intake deployed")
        self._deployed = True
        self.intake_deploy_motor.set(0.0)
        self.done()

    @magicbot.state
    def timed_out(self) -> None:
        self.logger.error("Intake deploy timed out!")
        self.intake_deploy_motor.set(0.0)
        self.done()

    def encoderPositionRotations(self) -> phoenix6.units.rotation:
        return self.intake_deploy_encoder.get_position().value

    def _logData(self) -> None:
        self.data_logger.logDouble(
            "/components/intake/deploy_encoder/position_rotations",
            self.encoderPositionRotations(),
        )
        datalog.logPrimaryMotorData(
            self.data_logger,
            "/components/intake/deploy_motor",
            self.intake_deploy_motor,
            position=True,
        )
        # Log the rest of the data at a slower frequency.
        if self._log_timer.advanceIfElapsed(1.0):
            datalog.logSecondaryMotorData(
                self.data_logger,
                "/components/intake/deploy_motor",
                self.intake_deploy_motor,
            )
