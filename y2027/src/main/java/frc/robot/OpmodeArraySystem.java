package frc.robot;

import frc.framework.OpMode;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

/**
 * A demo of arrays of non-primitive objects with the logging system
 */
public class OpmodeArraySystem implements Node {
	/**
	 * An array of opmodes that may change in length
	 */
	public static final ValueIdentifier<OpMode[]> OPMODE_ARRAY = ValueIdentifier.get("/opmode_array", OpMode[].class);
	
	@Override
	public void configure(NodeInformation information) {
		information.createsOutput(OPMODE_ARRAY);
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		if (controllerState == null) { return; }
		
		if (controllerState.getAxis(ControllerState.Axis.LeftThumbstickX) < 0) {
			update.setValue(OPMODE_ARRAY, new OpMode[]{OpMode.Test});
		} else {
			update.setValue(OPMODE_ARRAY, new OpMode[]{OpMode.Test, OpMode.Teleop});
		}
	}
}
