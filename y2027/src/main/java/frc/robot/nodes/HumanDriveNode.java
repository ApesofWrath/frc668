package frc.robot.nodes;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.ControllerState.Axis;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.Priority;

import static edu.wpi.first.units.Units.*;

public class HumanDriveNode implements Node {
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
		information.createsOutput(DrivetrainNode.WANTED_SPEEDS);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		if (controllerState == null) {
			update.setValue(DrivetrainNode.WANTED_SPEEDS, new ChassisSpeeds(), Priority.DriverInput); // Don't move if the controller is not connected
			return;
		}
		
		double driveSpeed = FeetPerSecond.of(3).in(MetersPerSecond); // 19.8 feet per second when we feel comfortable
		double rotateSpeed = RotationsPerSecond.of(1).in(RadiansPerSecond); // 2 rotations per second when we feel comfortable
		
		ChassisSpeeds speed = new ChassisSpeeds();
		
		speed.vxMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickY) * driveSpeed;
		speed.vyMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickX) * driveSpeed;
		speed.omegaRadiansPerSecond = controllerState.getAxis(Axis.RightThumbstickX) * rotateSpeed;
		
		update.setValue(DrivetrainNode.WANTED_SPEEDS, speed, Priority.DriverInput);
	}
	
}
