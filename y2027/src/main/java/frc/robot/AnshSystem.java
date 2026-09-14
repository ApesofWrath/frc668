package frc.robot;

import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

public class AnshSystem implements System {
	@Override
	public void configure(SystemInformation information) {
		information.createsOutput(NameExtenderSystem.NAME_TO_EXTEND_VALUE);
		information.recievesInput(UserInputInformation.CONTROLLER_INPUT);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT);
		
		if (controllerState.getAxis(ControllerState.Axis.LeftThumbstickX) < 0f) {
			update.setValue(NameExtenderSystem.NAME_TO_EXTEND_VALUE, "LeftAnsh");
		} else {
			update.setValue(NameExtenderSystem.NAME_TO_EXTEND_VALUE, "Ansh");
		}
	}
}
