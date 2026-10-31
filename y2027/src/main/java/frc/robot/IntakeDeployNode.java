package frc.robot;

import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.Motors;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;




public class IntakeDeployNode implements Node {

	double deployedPosition = 0.3;
	double undeployedPosition = 0;

	public static ValueIdentifier<Boolean> REQUEST_INTAKE_DEPLOY = ValueIdentifier.get(
			"/intake/request_intake_deploy", Boolean.class
	);
	
	@Override
	public void configure(NodeInformation information){
		information.receivesInput(REQUEST_INTAKE_DEPLOY);
//		information.receivesInput(Motors.positionIdentifier(MotorId.IntakeDeployer)); idk if this is necessary, probably not?

		information.createsOutput(Motors.controlIdentifier(MotorId.IntakeDeployer));
	}
	
	@Override
	public void update(NodeUpdateHelper update){
		boolean shouldDeploy = update.getValue(REQUEST_INTAKE_DEPLOY).orDefault(true, 9999999);

		double motorPos = (double) update.getValue(Motors.positionIdentifier(MotorId.IntakeDeployer)).orDefault(0.0, 10000);

		if (shouldDeploy){
			if (motorPos >= .3) {
				return;
			}
			update.setValue(Motors.controlIdentifier(MotorId.IntakeDeployer), MotorControl.voltage(.2));
		}
		if (!shouldDeploy){
			if (motorPos < 0) {
				return;
			}
			update.setValue(Motors.controlIdentifier(MotorId.IntakeDeployer), MotorControl.voltage(-.2));
		}
	}
}
