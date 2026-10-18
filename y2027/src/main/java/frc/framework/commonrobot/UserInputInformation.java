package frc.framework.commonrobot;

import frc.framework.nodes.ValueIdentifier;

/**
 * Contains various common ValueIdentifiers for I/O
 */
public class UserInputInformation {
	/**
	 * Represents the current state of the controller
	 */
	public static final ValueIdentifier<ControllerState> CONTROLLER_INPUT = ValueIdentifier.get(
		"/controller/state",
		ControllerState.class
	).replayed();
}
