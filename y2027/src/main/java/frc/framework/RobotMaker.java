package frc.framework;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.commonrobot.PhoenixMotorInputsNode;
import frc.framework.commonrobot.PhoenixMotorOutputsNode;
import frc.framework.nodes.NodesManager;

public interface RobotMaker {
	/**
	 * Add the various nodes pertaining to a given motor
	 *
	 * @param nodesManager The nodes manager to add the motors to
	 * @param id           The ID of the motor to add
	 * @param canId        The integer ID on the can-bus of the motor to add
	 * @param constants    The tuner constants for the motor
	 */
	default void addMotor(NodesManager nodesManager, Enum<?> id, int canId, PIDConstants constants) {
		// In simulation, this would have a special simulated motor system
		TalonFX talonFX = new TalonFX(canId);
		
		TalonFXConfiguration configs = new TalonFXConfiguration();
		
		configs.Slot0.kP = constants.getP();
		configs.Slot0.kI = constants.getI();
		configs.Slot0.kD = constants.getD();
		configs.Slot0.kG = constants.getG();
		
		talonFX.getConfigurator().apply(configs);
		
		nodesManager.addNode(new PhoenixMotorInputsNode(talonFX, id));
		nodesManager.addNode(new PhoenixMotorOutputsNode(talonFX, id));
	}
	
	
	/**
	 * Add the nodes relating to all subnodes, regardless of whether this is simulated or real.
	 *
	 * @param manager The NodesManager to add the nodes to
	 */
	void setupFullRobot(NodesManager manager);
	
	/**
	 * Add the nodes relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The NodesManager to add the nodes to
	 */
	void setupRealHardware(NodesManager manager);
	
	/**
	 * Add the nodes relating to simulation, whether it's a replay or not.
	 *
	 * @param manager The NodesManager to add the nodes to
	 */
	void setupSimulation(NodesManager manager);
}
