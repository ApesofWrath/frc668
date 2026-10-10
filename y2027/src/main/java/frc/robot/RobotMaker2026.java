package frc.robot;

import frc.framework.RobotMaker;
import frc.framework.builtinsystems.ControllerSystem;
import frc.framework.systems.SystemsManager;

/**
 * Sets up the 2026 Apes of Wrath robot
 */
public class RobotMaker2026 implements RobotMaker {
	@Override
	public void setupFullRobot(SystemsManager manager) {
		manager.addSystem(new HelloSpeakerSystem());
		manager.addSystem(new AnshSystem());
		manager.addSystem(new NameExtenderSystem());
		manager.addSystem(new ControllerSystem());
	}
	
	@Override
	public void setupRealHardware(SystemsManager manager) {
		
	}
	
	@Override
	public void setupSimulation(SystemsManager manager) {
		
	}
}
