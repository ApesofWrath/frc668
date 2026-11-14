package frc.robot;

import frc.framework.RobotMaker;
import frc.framework.builtinnodes.ControllerNode;
import frc.framework.nodes.NodesManager;
import frc.robot.nodes.DrivetrainNode;
import frc.robot.nodes.HumanDriveNode;

/**
 * Sets up the 2026 Apes of Wrath robot
 */
public class RobotMaker2026 implements RobotMaker {
	@Override
	public void setupFullRobot(NodesManager manager) {
		manager.addNode(new HumanDriveNode());
		manager.addNode(new DrivetrainNode());
		manager.addNode(new ControllerNode());
		manager.addNode(new OpmodeArraySystem());
	}
	
	@Override
	public void setupRealHardware(NodesManager manager) {
		
	}
	
	@Override
	public void setupSimulation(NodesManager manager) {
		
	}
}
