package frc.robot.nodes;

import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.commonrobot.ControllerState.Button;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;

public class HumanIntakeNode implements Node {
	
	
	@Override
	public void configure(NodeInformation information){
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
		
		information.createsOutput(IntakeDeployNode.REQUEST_DEPLOY_INTAKE);
		information.createsOutput(IntakeNode.REQUEST_SPIN_INTAKE);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		boolean requestSpinIntake = false;
		
		if (controllerState == null) {
			return;
		}

		if (controllerState.getButtonState(Button.A)){
			update.setValue(IntakeDeployNode.REQUEST_DEPLOY_INTAKE, true);
		}
		if (controllerState.getButtonState(Button.B)){
			update.setValue(IntakeDeployNode.REQUEST_DEPLOY_INTAKE, false);
		}
		if (controllerState.getButtonState(Button.X)){
			requestSpinIntake = true;
		}
		update.setValue(IntakeNode.REQUEST_SPIN_INTAKE, requestSpinIntake);
		
	}
}
