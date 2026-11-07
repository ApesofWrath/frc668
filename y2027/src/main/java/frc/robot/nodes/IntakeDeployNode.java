package frc.robot.nodes;


import frc.framework.OpMode;
import frc.framework.cache.MemoizationCacheStrategy;
import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.Motors;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import frc.robot.MotorId;




public class IntakeDeployNode implements Node {

	double deployedPosition = 0.3; /*the deployed and undeployed states are based off of what phoenix tuner says the
								     motor positions are when the intake is folded in or out, these are
									 rounded values and should be updated */
	double unDeployedPosition = 0.1; // these should probably be intake constants
	
	public static ValueIdentifier<Boolean> REQUEST_DEPLOY_INTAKE = ValueIdentifier.get(
		"/drivetrain/request_deploy_intake",
		Boolean.class
	);


	@Override
	public void configure(NodeInformation information){
		information.cacheStrategy = new MemoizationCacheStrategy();
		
		information.receivesInput(Motors.positionIdentifier(MotorId.IntakeDeployer)); 
		information.receivesInput(RobotInformation.OPMODE_VALUE);
		information.receivesInput(REQUEST_DEPLOY_INTAKE);
		
		information.createsOutput(Motors.controlIdentifier(MotorId.IntakeDeployer));
	}
	
	@Override
	public void update(NodeUpdateHelper update){

		boolean requestDeploy = update.getValue(REQUEST_DEPLOY_INTAKE).orDefault(false);

		double motorPos = (double) update.getValue(Motors.positionIdentifier(MotorId.IntakeDeployer)).orDefault(0.0, 10000);

		OpMode opMode = update.getValue(RobotInformation.OPMODE_VALUE).orNull();

		if (opMode != OpMode.Teleop){
			update.setValue(Motors.controlIdentifier(MotorId.IntakeDeployer), MotorControl.velocity(0));
			return;
		}

		if(requestDeploy && (motorPos<deployedPosition)){
			update.setValue(Motors.controlIdentifier(MotorId.IntakeDeployer), MotorControl.velocity(.2));
		}
		else if (!requestDeploy && (motorPos>unDeployedPosition)){
			update.setValue(Motors.controlIdentifier(MotorId.IntakeDeployer), MotorControl.velocity(.2));
		}
		
		
		
	}
}
