package frc.framework.builtinnodes;

import frc.framework.OpMode;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.Priority;

/**
 * A node that outputs HAL information about a robot, namely the opmode.
 */
public class HALRobotInformationNode implements Node {
	/**
	 * The current operating mode of the robot
	 */
	public OpMode opMode;
	
	@Override
	public void configure(NodeInformation information) {
		information.createsOutput(RobotInformation.OPMODE_VALUE);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		update.setValue(RobotInformation.OPMODE_VALUE, opMode, Priority.Safety);
	}
}
