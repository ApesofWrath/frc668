package frc.framework;

/**
 * The current operating mode of the robot
 */
public enum OpMode {
	/**
	 * The robot is not enabled
	 */
	Disabled,
	/**
	 * The game is in the initial autonomous period
	 */
	Autonomous,
	/**
	 * The game is in the driver controlled period
	 */
	Teleop,
	/**
	 * The robot is in test mode
	 */
	Test,
}
