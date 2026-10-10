package frc.framework.systems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.framework.OpMode;
import frc.framework.RobotMaker;
import frc.framework.builtinsystems.HALRobotInformationSystem;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.commonrobot.PhoenixMotorInputsSystem;
import frc.framework.commonrobot.PhoenixMotorOutputsSystem;

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
	
	public SystemsRobot(RobotMaker maker) {
		robotMaker = maker;
	}
	
	@Override
	public void autonomousPeriodic() {
		update(OpMode.Autonomous);
	}
	
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
			robotMaker.setupFullRobot(systemsManager);
			if (isReal()) {
				robotMaker.setupRealHardware(systemsManager);
			} else {
				robotMaker.setupSimulation(systemsManager);
			}
			systemsManager.addSystem(robotInformationSystem);
			isConfigured = true;
		}
		robotInformationSystem.opMode = mode;
		systemsManager.update(new Date().getTime());
		systemsManager.publishValuesToNetworkTables();
	}
}
