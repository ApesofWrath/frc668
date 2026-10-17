package frc.framework.commonrobot;

/**
 * A type of how a motor should accelerate
 */
public enum MotorControlType {
	/**
	 * The motor control to attempt to get to a position
	 */
	Position,
	/**
	 * The motor control to accelerate to a given velocity
	 */
	Velocity,
	/**
	 * The motor control to apply a given acceleration to a motor
	 */
	Voltage
}
