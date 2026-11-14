package frc.framework.simulation;

import org.mujoco.MuJoCoModelManager;
import org.mujoco.MuJoCoLib.mjModel_;

import edu.wpi.first.wpilibj.Filesystem;


public class Simulator {
	private static Simulator globalSim;
	
	public static Simulator getInstance() {
		if (globalSim == null) {
			globalSim = new Simulator();
			globalSim.Init();
		}
		return globalSim;
	}
	
	public void Exit() {
		
	}
	
	public void Init() {
		//when the simulation starts, start the simulation
		var modelManager = new MuJoCoModelManager(
			Filesystem.getDeployDirectory().toPath().resolve("model.mjml").toFile()
		);
		mjModel_ model = modelManager.getModel();
		
		System.out.println("asdasd");
	}
	
	public void Run() {
		
	}
}
