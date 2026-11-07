package frc.robot;

import com.ctre.phoenix6.CANBus;
import frc.framework.RobotMaker;
import frc.framework.builtinnodes.ControllerNode;
import frc.framework.nodes.NodesManager;
import frc.robot.nodes.DrivetrainNode;
import frc.robot.nodes.HumanDriveNode;
import frc.robot.nodes.pose.LimelightNode;
import frc.robot.nodes.pose.PigeonNode;

/**
 * Sets up the 2026 Apes of Wrath robot
 */
public class RobotMaker2026 implements RobotMaker {
	@Override
	public void setupFullRobot(NodesManager manager) {
		manager.addNode(new HumanDriveNode());
		manager.addNode(new DrivetrainNode());
		manager.addNode(new ControllerNode());
		manager.addNode(new PigeonNode(22, new CANBus("swerve")));
		manager.addNode(new LimelightNode("upfl"));
	}
	
	@Override
	public void setupRealHardware(NodesManager manager) {
		
	}
	
	@Override
	public void setupSimulation(NodesManager manager) {
		
	}
}
