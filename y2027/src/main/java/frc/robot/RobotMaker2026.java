package frc.robot;

import frc.framework.RobotMaker;
import frc.framework.builtinsystems.ControllerSystem;
import frc.framework.systems.SystemsManager;
import frc.robot.systems.DrivetrainSubsystem;
import frc.robot.systems.HumanDriveSystem;

/**
 * Sets up the 2026 Apes of Wrath robot
 */
public class RobotMaker2026 implements RobotMaker {
	@Override
	public void setupFullRobot(SystemsManager manager) {
		manager.addSystem(new HumanDriveSystem());
		manager.addSystem(new DrivetrainSubsystem());
		manager.addSystem(new ControllerSystem());
	}
	
	@Override
	public void setupRealHardware(SystemsManager manager) {
		
	}
	
	@Override
	public void setupSimulation(SystemsManager manager) {
		
	}
}
