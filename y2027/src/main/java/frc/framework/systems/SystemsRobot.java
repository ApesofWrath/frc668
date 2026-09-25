package frc.framework.systems;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.framework.OpMode;
import frc.framework.builtinsystems.HALRobotInformationSystem;
import frc.framework.logging.LogFrame;
import frc.framework.logging.LogWriter;

import java.util.Date;

/**
 * A utility class that manages the HAL information bridge and framework management for an FRC robot.
 */
public abstract class SystemsRobot extends TimedRobot {
	private final HALRobotInformationSystem robotInformationSystem = new HALRobotInformationSystem();
	private final LogWriter logger = LogWriter.open(LogWriter.getLogPath());
	/**
	 * The current systems framework manager
	 */
	public SystemsManager systemsManager = new SystemsManager();
	private boolean isConfigured = false;
	
	@Override
	public void autonomousPeriodic() {
		update(OpMode.Autonomous);
	}
	
	/**
	 * The method that adds the systems for a given robot
	 */
	public abstract void configure();
	
	@Override
	public void disabledPeriodic() {
		update(OpMode.Disabled);
	}
	
	@Override
	public void teleopPeriodic() {
		update(OpMode.Teleop);
	}
	
	@Override
	public void testPeriodic() {
		update(OpMode.Test);
	}
	
	/**
	 * Tick a robot
	 *
	 * @param mode The mode of the robot
	 */
	public void update(OpMode mode) {
		if (!isConfigured) {
			configure();
			isConfigured = true;
		}
		robotInformationSystem.opMode = mode;
		
		LogFrame frame = new LogFrame();
		
		long time = new Date().getTime();
		
		systemsManager.update(time, frame);
		logger.writeFrame(frame, time);
		
		systemsManager.publishValuesToNetworkTables();
	}
}
