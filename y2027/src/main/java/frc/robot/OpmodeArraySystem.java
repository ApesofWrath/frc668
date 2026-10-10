package frc.robot;

import frc.framework.OpMode;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.framework.systems.ValueIdentifier;

/**
 * A demo of arrays of non-primitive objects with the logging system
 */
public class OpmodeArraySystem implements System {
	/**
	 * An array of opmodes that may change in length
	 */
	public static final ValueIdentifier<OpMode[]> OPMODE_ARRAY = ValueIdentifier.get("/opmode_array", new OpMode[0]);
	
	@Override
	public void configure(SystemInformation information) {
		information.createsOutput(OPMODE_ARRAY);
		information.recievesInput(UserInputInformation.CONTROLLER_INPUT);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT);
		
		if (controllerState.getAxis(ControllerState.Axis.LeftThumbstickX) < 0) {
			update.setValue(OPMODE_ARRAY, new OpMode[]{OpMode.Test});
		} else {
			update.setValue(OPMODE_ARRAY, new OpMode[]{OpMode.Test, OpMode.Teleop});
		}
	}
}
