package frc.robot;

import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;

/**
 * Writes the name Ansh or LeftAnsh to {@link NameExtenderNode#NAME_TO_EXTEND_VALUE}
 */
public class AnshNode implements Node {
	@Override
	public void configure(NodeInformation information) {
		information.createsOutput(NameExtenderNode.NAME_TO_EXTEND_VALUE);
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		if (controllerState == null) { return; }
		
		if (controllerState.getAxis(ControllerState.Axis.LeftThumbstickX) < 0f) {
			update.setValue(NameExtenderNode.NAME_TO_EXTEND_VALUE, "LeftAnsh");
		} else {
			update.setValue(NameExtenderNode.NAME_TO_EXTEND_VALUE, "Ansh");
		}
	}
}
