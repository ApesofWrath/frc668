package frc.framework;

import frc.framework.systems.SystemsManager;

public interface RobotMaker {
	/**
	 * Add the systems relating to all subsystems, regardless of whether this is simulated or real.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupFullRobot(SystemsManager manager);
	
	
	/**
	 * Add the systems relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupRealHardware(SystemsManager manager);
	
	/**
	 * Add the systems relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupSimulation(SystemsManager manager);
}
