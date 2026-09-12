package frc.framework.systems;

import java.util.Date;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.framework.OpMode;
import frc.framework.builtinsystems.HALRobotInformationSystem;

public abstract class SystemsRobot extends TimedRobot {
	public SystemsManager systemsManager = new SystemsManager();
	public HALRobotInformationSystem robotInformationSystem =
			new HALRobotInformationSystem();
	private boolean isConfigured = false;

	public void update(OpMode mode) {
		if (!isConfigured) {
			configure();
			isConfigured = true;
		}
		robotInformationSystem.opMode = mode;
		systemsManager.update(new Date().getTime());
		systemsManager.publishValuesToNetworkTables();
	}

	public abstract void configure();

	@Override
	public void autonomousPeriodic() {
		update(OpMode.Autonomous);
	}

	@Override
	public void teleopPeriodic() {
		update(OpMode.Teleop);
	}

	@Override
	public void disabledPeriodic() {
		update(OpMode.Disabled);
	}

	@Override
	public void testPeriodic() {
		update(OpMode.Test);
	}
}
