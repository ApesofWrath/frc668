package frc.robot;

import frc.framework.RobotMaker;
import frc.framework.builtinnodes.ControllerNode;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.nodes.NodesManager;

/**
 * Sets up the 2026 Apes of Wrath robot
 */
public class RobotMaker2026 implements RobotMaker {
	@Override
	public void setupFullRobot(NodesManager manager) {
		manager.addNode(new HelloSpeakerNode());
		manager.addNode(new AnshNode());
		manager.addNode(new NameExtenderNode());
		manager.addNode(new ControllerNode());
		addMotor(manager, MotorId.IntakeRoller, 0, new PIDConstants().withP(1).withI(0).withD(0));
	}
	
	@Override
	public void setupRealHardware(NodesManager manager) {
		
	}
	
	@Override
	public void setupSimulation(NodesManager manager) {
		
	}
}
