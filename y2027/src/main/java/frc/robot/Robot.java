package frc.robot;

import frc.framework.systems.SystemsRobot;

public class Robot extends SystemsRobot {
	@Override
	public void configure() {
		systemsManager.addSystem(new HelloSpeakerSystem());
		systemsManager.addSystem(new AnshSystem());
		systemsManager.addSystem(new NameExtenderSystem());
	}
}
