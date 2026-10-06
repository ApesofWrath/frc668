package frc.framework.systems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.framework.OpMode;
import frc.framework.builtinsystems.HALRobotInformationSystem;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.commonrobot.PhoenixMotorInputsSystem;
import frc.framework.commonrobot.PhoenixMotorOutputsSystem;

import java.util.Date;

/**
 * A utility class that manages the HAL information bridge and framework management for an FRC robot.
 */
public abstract class SystemsRobot extends TimedRobot {
	private final HALRobotInformationSystem robotInformationSystem = new HALRobotInformationSystem();
	/**
	 * The current systems framework manager
	 */
	public SystemsManager systemsManager = new SystemsManager();
	private boolean isConfigured = false;
	
	/**
	 * Add the various systems pertaining to a given motor
	 *
	 * @param id        The ID of the motor to add
	 * @param canId     The integer ID on the can-bus of the motor to add
	 * @param constants The tuner constants for the motor
	 */
	protected void addMotor(Enum<?> id, int canId, PIDConstants constants) {
		// In simulation, this would have a special simulated motor system
		TalonFX talonFX = new TalonFX(canId);
		
		TalonFXConfiguration configs = new TalonFXConfiguration();
		
		configs.Slot0.kP = constants.getP();
		configs.Slot0.kI = constants.getI();
		configs.Slot0.kD = constants.getD();
		configs.Slot0.kG = constants.getG();
		
		talonFX.getConfigurator().apply(configs);
		
		systemsManager.addSystem(new PhoenixMotorInputsSystem(talonFX, id));
		systemsManager.addSystem(new PhoenixMotorOutputsSystem(talonFX, id));
	}
	
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
			systemsManager.addSystem(robotInformationSystem);
			isConfigured = true;
		}
		robotInformationSystem.opMode = mode;
		systemsManager.update(new Date().getTime());
		systemsManager.publishValuesToNetworkTables();
	}
}
