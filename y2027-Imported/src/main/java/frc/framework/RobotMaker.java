package frc.framework;

import frc.framework.nodes.NodesManager;

public interface RobotMaker {
	/**
	 * Add the nodes relating to all subnodes, regardless of whether this is simulated or real.
	 *
	 * @param manager The NodesManager to add the nodes to
	 */
	void setupFullRobot(NodesManager manager);
	
	
	/**
	 * Add the nodes relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The NodesManager to add the nodes to
	 */
	void setupRealHardware(NodesManager manager);
	
	/**
	 * Add the nodes relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The NodesManager to add the nodes to
	 */
	void setupSimulation(NodesManager manager);
}
