package frc.framework.commonrobot;

import frc.framework.systems.ValueIdentifier;

public class UserInputInformation {
	public static final ValueIdentifier<ControllerState> CONTROLLER_INPUT = new ValueIdentifier<ControllerState>(
		"/controller/state",
		new ControllerState()
	);
}
