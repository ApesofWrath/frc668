package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.framework.builtinsystems.ControllerSystem;
import frc.framework.commonrobot.PIDConstants;
import frc.framework.commonrobot.PhoenixMotorInputsSystem;
import frc.framework.commonrobot.PhoenixMotorOutputsSystem;
import frc.framework.systems.SystemsRobot;

/**
 * An example robot
 */
public class Robot extends SystemsRobot {
	private void addMotor(MotorId id, int canId, PIDConstants constants) {
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
	public void configure() {
		systemsManager.addSystem(new HelloSpeakerSystem());
		systemsManager.addSystem(new AnshSystem());
		systemsManager.addSystem(new NameExtenderSystem());
		systemsManager.addSystem(new ControllerSystem());
		
		systemsManager.addSystem(new IntakeSystem());
		addMotor(MotorId.IntakeRoller, 0, new PIDConstants().withP(1).withI(0).withD(0));
	}
}
