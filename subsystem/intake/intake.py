import phoenix6
import magicbot
import constants
from phoenix6.configs import config_groups
from phoenix6.controls import PositionVoltage

class Intake:
    """Intake component

    This class drives the intake motors that pick up fuel into the hopper.
    """

    robot_constants: constants.RobotConstants
    intake_roller_motor: phoenix6.hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the intake.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        intake_constants = self.robot_constants.intake
        self._intake_roller_motor_speed = 0.0

        self.intake_roller_motor.configurator.apply(
            phoenix6.configs.TalonFXConfiguration().with_motor_output(
                phoenix6.configs.MotorOutputConfigs().with_inverted(
                    intake_constants.intake_roller_motor_inverted
                )
            )
        )

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.intake_roller_motor.set(self._intake_roller_motor_speed)

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._intake_roller_motor_speed = 0.0

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._intake_roller_motor_speed = 0.0

    def setMotorSpeed(self, speed: float) -> None:
        self._intake_roller_motor_speed = speed