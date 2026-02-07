import magicbot
import phoenix6
import wpilib

from subsystem import shooter


class FlywheelTuner:
    """Component for tuning the flywheel gains.

    It sets up tunable gains over network tables so they can be easily modified
    on AdvantageScope. It also provides a settable flywheel target velocity.
    """

    target_rps = magicbot.tunable(shooter.constants.FLYWHEEL_ENABLE_RPS)
    k_s = magicbot.tunable(shooter.constants.FLYWHEEL_K_S)
    k_v = magicbot.tunable(shooter.constants.FLYWHEEL_K_V)
    k_a = magicbot.tunable(shooter.constants.FLYWHEEL_K_A)
    k_p = magicbot.tunable(shooter.constants.FLYWHEEL_K_P)
    k_i = magicbot.tunable(shooter.constants.FLYWHEEL_K_I)
    k_d = magicbot.tunable(shooter.constants.FLYWHEEL_K_D)

    def setup(self) -> None:
        """Set up initial state for the flywheel tuner.

        This method is called after createObjects has been called in the main
        robot class, and after all components have been created.
        """
        self.last_k_s = shooter.constants.FLYWHEEL_K_S
        self.last_k_v = shooter.constants.FLYWHEEL_K_V
        self.last_k_a = shooter.constants.FLYWHEEL_K_A
        self.last_k_p = shooter.constants.FLYWHEEL_K_P
        self.last_k_i = shooter.constants.FLYWHEEL_K_I
        self.last_k_d = shooter.constants.FLYWHEEL_K_D

    def execute(self) -> None:
        """Cache the latest gains.

        This method is called at the end of the control loop, so they are
        updated after any calls to gainsChanged from teleopPeriodic.
        """
        self.last_k_s = self.k_s
        self.last_k_v = self.k_v
        self.last_k_a = self.k_a
        self.last_k_p = self.k_p
        self.last_k_i = self.k_i
        self.last_k_d = self.k_d

    def gainsChanged(self) -> bool:
        """Detect if any of the gains changed.

        Returns:
            True if any of the gains changed, False otherwise.
        """
        return (
            self.k_s != self.last_k_s
            or self.k_v != self.last_k_v
            or self.k_a != self.last_k_a
            or self.k_p != self.last_k_p
            or self.k_i != self.last_k_i
            or self.k_d != self.last_k_d
        )

    def gains(self) -> phoenix6.configs.config_groups.Slot0Configs:
        """Get the current gains.

        Returns:
            The current gains as a Slot0Configs object.
        """
        return (
            phoenix6.configs.config_groups.Slot0Configs()
            .with_k_s(self.k_s)
            .with_k_v(self.k_v)
            .with_k_a(self.k_a)
            .with_k_p(self.k_p)
            .with_k_i(self.k_i)
            .with_k_d(self.k_d)
        )

    def get_target_rps(self) -> float:
        """Get the current target velocity for the flywheel.

        Returns:
            The target velocity in rotations per second.
        """
        return self.target_rps


class FlywheelTunerRobot(magicbot.MagicRobot):
    """Robot class for tuning the flywheel.

    This contains just enough components to be able to tune and test the
    flywheel's performance under load.
    """

    flywheel_tuner: FlywheelTuner
    flywheel: shooter.Flywheel
    hopper: shooter.Hopper
    indexer: shooter.Indexer

    def createObjects(self) -> None:
        """Create and initialize robot objects."""
        self.operator_controller = wpilib.XboxController(1)

        # Flywheel motor and encoder.
        self.flywheel_motor = phoenix6.hardware.TalonFX(
            shooter.constants.FLYWHEEL_MOTOR_CAN_ID,
            "Shooter",
        )
        self.flywheel_encoder = phoenix6.hardware.CANcoder(
            shooter.constants.FLYWHEEL_ENCODER_CAN_ID,
            "Shooter",
        )

        # Hopper motors.
        self.hopper_left_motor = phoenix6.hardware.TalonFX(
            shooter.constants.HOPPER_LEFT_MOTOR_CAN_ID,
            "rio",
        )
        self.hopper_right_motor = phoenix6.hardware.TalonFX(
            shooter.constants.HOPPER_RIGHT_MOTOR_CAN_ID,
            "rio",
        )

        # Indexer motors.
        self.indexer_bottom_motor = phoenix6.hardware.TalonFX(
            shooter.constants.INDEXER_BOTTOM_MOTOR_CAN_ID,
            "Shooter",
        )
        self.indexer_top_motor = phoenix6.hardware.TalonFX(
            shooter.constants.INDEXER_TOP_MOTOR_CAN_ID,
            "Shooter",
        )

    def teleopPeriodic(self) -> None:
        """Run during teleoperated mode.

        This is called periodically at a regular rate when the robot is in
        teleoperated mode. This code executes before the `execute` method of all
        components are called.

        If you want this method to be called in autonomous mode, set
        `use_teleop_in_autonomous=True` in this class' instance.
        """
        # Set the target velocity for the flywheel.
        self.flywheel.setTargetRps(self.flywheel_tuner.get_target_rps())
        # If any of the gains changed, register the new gains with the flywheel.
        if self.flywheel_tuner.gainsChanged():
            self.flywheel._setSlot0Configs(self.flywheel_tuner.gains())

        # Drive the hopper and indexer motors when the right bumper is pressed.
        if self.operator_controller.getRightBumper():
            self.indexer.setMotorSpeedRps(1.0)
            self.hopper.setMotorSpeedRps(1.0)
        else:
            self.indexer.setMotorSpeedRps(0.0)
            self.hopper.setMotorSpeedRps(0.0)
