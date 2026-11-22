package frc.robot.nodes;

import org.wpilib.math.kinematics.ChassisVelocities;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.ControllerState.Axis;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.Priority;
import frc.robot.GameController;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import static org.wpilib.units.Units.*;

import org.wpilib.driverstation.XboxController;

/**
 * Handles the conversion of controller input to desired chassis speeds
 */
public class HumanDriveNode implements Node {

	private GameController controller = new GameController(new XboxController(0));
		private final TalonFX motor = new TalonFX(0, new CANBus("CANivoreName"));
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
		information.createsOutput(DrivetrainNode.WANTED_SPEEDS);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		controller.checkButton();

		if (controllerState == null) {
			return;
		}
		
		double driveSpeed = FeetPerSecond.of(3).in(MetersPerSecond); // 19.8 feet per second when we feel comfortable
		double rotateSpeed = RotationsPerSecond.of(1).in(RadiansPerSecond); // 2 rotations per second when we feel comfortable
		
		ChassisVelocities speed = new ChassisVelocities(
			controllerState.getAxis(Axis.LeftThumbstickY) * driveSpeed,
			controllerState.getAxis(Axis.LeftThumbstickX) * driveSpeed,
			controllerState.getAxis(Axis.RightThumbstickX) * rotateSpeed
		);
		update.setValue(DrivetrainNode.WANTED_SPEEDS, speed, Priority.DriverInput);
	}
	
}
