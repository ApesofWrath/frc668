package frc.framework.nodes;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.framework.OpMode;
import frc.framework.RobotMaker;
import frc.framework.builtinnodes.HALRobotInformationNode;

import java.util.Date;

/**
 * A utility class that manages the HAL information bridge and framework management for an FRC robot.
 */
public class NodesRobot extends TimedRobot {
	private final HALRobotInformationNode robotInformationNode = new HALRobotInformationNode();
	private final RobotMaker robotMaker;
	/**
	 * The current systems framework manager
	 */
	public NodesManager nodesManager = new NodesManager();
	private boolean isConfigured = false;
	
	/**
	 * Create a SystemsRobot
	 *
	 * @param maker The template used to instantiate the robot systems
	 */
	public NodesRobot(RobotMaker maker) {
		robotMaker = maker;
	}
	
	@Override
	public void autonomousInit() {
		super.autonomousInit();
		maybeConfigure();
	}
	
	@Override
	public void autonomousPeriodic() {
		update(OpMode.Autonomous);
	}
	
	@Override
	public void disabledInit() {
		super.disabledInit();
		maybeConfigure();
	}
	
	@Override
	public void disabledPeriodic() {
		update(OpMode.Disabled);
	}
	
	private void maybeConfigure() {
		if (!isConfigured) {
			robotMaker.setupFullRobot(nodesManager);
			if (isReal()) {
				robotMaker.setupRealHardware(nodesManager);
			} else {
				robotMaker.setupSimulation(nodesManager);
			}
			nodesManager.addNode(robotInformationNode);
			isConfigured = true;
		}
	}
	
	@Override
	public void teleopInit() {
		super.teleopInit();
		maybeConfigure();
	}
	
	@Override
	public void teleopPeriodic() {
		update(OpMode.Teleop);
	}
	
	@Override
	public void testInit() {
		super.testInit();
		maybeConfigure();
	}
	
	@Override
	public void testPeriodic() {
		update(OpMode.Test);
	}
	
	/**
	 * Tick a robot
	 *
	 * @param mode The mode of the robot
	 */
	public void update(OpMode mode) {
		maybeConfigure();
		robotInformationNode.opMode = mode;
		nodesManager.update(new Date().getTime());
		nodesManager.publishValuesToNetworkTables();
	}
}
