package frc.framework;

import frc.framework.nodes.NodesManager;

public interface RobotMaker {
	/**
	 * Add the systems relating to all subsystems, regardless of whether this is simulated or real.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupFullRobot(NodesManager manager);
	
	
	/**
	 * Add the systems relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupRealHardware(NodesManager manager);
	
	/**
	 * Add the systems relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupSimulation(NodesManager manager);
}
