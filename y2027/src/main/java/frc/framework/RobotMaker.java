package frc.framework;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.commonrobot.PhoenixMotorInputsSystem;
import frc.framework.commonrobot.PhoenixMotorOutputsSystem;
import frc.framework.systems.SystemsManager;

public interface RobotMaker {
	/**
	 * Add the various systems pertaining to a given motor
	 *
	 * @param systemsManager The systems manager to add the motors to
	 * @param id             The ID of the motor to add
	 * @param canId          The integer ID on the can-bus of the motor to add
	 * @param constants      The tuner constants for the motor
	 */
	default void addMotor(SystemsManager systemsManager, Enum<?> id, int canId, PIDConstants constants) {
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
	
	
	/**
	 * Add the systems relating to all subsystems, regardless of whether this is simulated or real.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupFullRobot(SystemsManager manager);
	
	/**
	 * Add the systems relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupRealHardware(SystemsManager manager);
	
	/**
	 * Add the systems relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The SystemsManager to add the systems to
	 */
	void setupSimulation(SystemsManager manager);
}
