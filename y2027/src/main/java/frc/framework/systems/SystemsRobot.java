package frc.framework.systems;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.framework.OpMode;
import frc.framework.RobotMaker;
import frc.framework.builtinsystems.HALRobotInformationSystem;

import java.util.Date;

/**
 * A utility class that manages the HAL information bridge and framework management for an FRC robot.
 */
public class SystemsRobot extends TimedRobot {
	private final HALRobotInformationSystem robotInformationSystem = new HALRobotInformationSystem();
	private final RobotMaker robotMaker;
	/**
	 * The current systems framework manager
	 */
	public SystemsManager systemsManager = new SystemsManager();
	private boolean isConfigured = false;
	
	/**
	 * Create a SystemsRobot
	 *
	 * @param maker The template used to instantiate the robot systems
	 */
	public SystemsRobot(RobotMaker maker) {
		robotMaker = maker;
	}
	
	@Override
	public void autonomousInit() {
		super.autonomousInit();
		maybeConfigure();
	}
	
	@Override
	public void autonomousPeriodic() {
		update(OpMode.Autonomous);
	}
	
	@Override
	public void disabledInit() {
		super.disabledInit();
		maybeConfigure();
	}
	
	@Override
	public void disabledPeriodic() {
		update(OpMode.Disabled);
	}
	
	private void maybeConfigure() {
		if (!isConfigured) {
			robotMaker.setupFullRobot(systemsManager);
			if (isReal()) {
				robotMaker.setupRealHardware(systemsManager);
			} else {
				robotMaker.setupSimulation(systemsManager);
			}
			systemsManager.addSystem(robotInformationSystem);
			isConfigured = true;
		}
	}
	
	@Override
	public void teleopInit() {
		super.teleopInit();
		maybeConfigure();
	}
	
	@Override
	public void teleopPeriodic() {
		update(OpMode.Teleop);
	}
	
	@Override
	public void testInit() {
		super.testInit();
		maybeConfigure();
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
		maybeConfigure();
		robotInformationSystem.opMode = mode;
		systemsManager.update(new Date().getTime());
		systemsManager.publishValuesToNetworkTables();
	}
}
