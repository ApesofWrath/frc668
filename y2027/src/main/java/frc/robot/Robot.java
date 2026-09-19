package frc.robot;

import frc.framework.builtinsystems.ControllerSystem;
import frc.framework.systems.SystemsRobot;

/**
 * An example robot
 */
public class Robot extends SystemsRobot {
	@Override
	public void configure() {
		systemsManager.addSystem(new ControllerSystem());
	}
}
