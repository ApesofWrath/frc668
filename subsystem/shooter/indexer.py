import phoenix6


class Indexer:
    """Indexer component

    This class drives the indexer motors that feed fuel into the flywheel.
    """

    indexer_bottom_motor: phoenix6.hardware.TalonFX
    indexer_top_motor: phoenix6.hardware.TalonFX

    def setup(self) -> None:
        """Set up initial state for the indexer.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        # A value in the range [0, 1], representing the duty cycle.
        self._motor_speed: float = 0.0

    def execute(self) -> None:
        """Command the motors to the current speed.

        This method is called at the end of the control loop.
        """
        self.indexer_bottom_motor.set(self._motor_speed)
        self.indexer_top_motor.set(-self._motor_speed)

    def on_enable(self) -> None:
        """Reset to a "safe" state when the robot is enabled.

        This method is called when the robot enters autonomous, teleoperated, or
        test mode.
        """
        self._motor_speed = 0.0

    def on_disable(self) -> None:
        """Reset state when the robot is disabled.

        This method is called when the robot enters disabled mode.
        """
        self._motor_speed = 0.0

    def setMotorSpeed(self, speed: float) -> None:
        """Set the speed of the indexer motors.

        Args:
            speed: A value in the range [0, 1] representing the duty cycle to
                apply to the motor.
        """
        self._motor_speed = speed
