package frc.robot;

import frc.framework.builtinsystems.ControllerSystem;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.systems.SystemsRobot;

/**
 * An example robot
 */
public class Robot extends SystemsRobot {
	@Override
	public void configure() {
		systemsManager.addSystem(new HelloSpeakerSystem());
		systemsManager.addSystem(new AnshSystem());
		systemsManager.addSystem(new NameExtenderSystem());
		systemsManager.addSystem(new ControllerSystem());
		
		systemsManager.addSystem(new IntakeSystem());
		addMotor(MotorId.IntakeRoller, 0, new PIDConstants().withP(1).withI(0).withD(0));
	}
}
