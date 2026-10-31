package frc.robot;

import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;

public class HumanIntakeNode implements Node {
	
	
	@Override
	public void configure(NodeInformation information){
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
		
		information.createsOutput(IntakeDeployNode.REQUEST_INTAKE_DEPLOY);
		
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		boolean controllerShouldDeploy = update.getValue(IntakeDeployNode.REQUEST_INTAKE_DEPLOY).orDefault(true, 9999999);
		
		if (controllerState == null) {
			return;
		}
		
		if (controllerState.getButtonState(ControllerState.Button.A)) {
			update.setValue(IntakeDeployNode.REQUEST_INTAKE_DEPLOY, true);
		} else if (controllerState.getButtonState(ControllerState.Button.B)) {
			update.setValue(IntakeDeployNode.REQUEST_INTAKE_DEPLOY, false);
		}
	}
}
