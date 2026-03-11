import magicbot
import phoenix6
import wpilib

import constants


class IntakeDeployer(magicbot.StateMachine):
    robot_constants: constants.RobotConstants
    intake_deploy_motor: phoenix6.hardware.TalonFX
    intake_deploy_encoder: phoenix6.hardware.CANcoder

    def __init__(self):
        self._first_spike = True
        self._timer = wpilib.Timer()
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
                .with_neutral_mode(phoenix6.signals.NeutralModeValue.BRAKE)
            )
            .with_current_limits(
                phoenix6.configs.CurrentLimitsConfigs()
                .with_supply_current_limit(self.robot_constants.intake.supply_current_limit)
                .with_supply_current_limit_enable(True)
            )
        )

    def deploy(self) -> None:
        self.engage()
        self.logger.info(self.current_state)

    @magicbot.state(first=True, must_finish=True)
    def deploying(self, state_tm) -> None:
        self._deployed = False
        self.intake_deploy_motor.set(0.25)
        if self.intake_deploy_motor.get_stator_current().value >= 12.0:
            if self._first_spike:
                self._first_spike = False
                self._timer.reset()
                self._timer.start()
            else:
                if self._timer.hasElapsed(0.25):
                    self._first_spike = True
                    self._timer.stop()
                    self._timer.reset()
                    self.next_state("deployed")
        elif state_tm >= 10.0:
            self.next_state("timeout")
        else:
            self.logger.info("Deploying")

    @magicbot.state()
    def deployed(self):
        self.intake_deploy_motor.set(0.0)
        self._deployed = True
        self.done()
        self._timer.stop()
        self._timer.reset()
        self.logger.info("Success: Intake deployed")

    @magicbot.state()
    def timeout(self) -> None:
        self.logger.warning("Intake deploy timed out!")
        self.intake_deploy_motor.set(0.0)
        self.done()