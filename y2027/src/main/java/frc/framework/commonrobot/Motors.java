package frc.framework.commonrobot;

import frc.framework.systems.ValueIdentifier;

/**
 * A utility class for working with motors.
 */
public class Motors {
	/**
	 * Gets a value identifier for a motors control
	 *
	 * @param motorId A string identifier to distinguish the motor from others
	 *
	 * @return The motors control value identifier
	 */
	public static ValueIdentifier<MotorControl> controlIdentifier(Enum<?> motorId) {
		return ValueIdentifier.get("/motors/" + motorId.name() + "/control", MotorControl.voltage(0));
	}
	
	/**
	 * Gets a value identifier for a motors position
	 *
	 * @param motorId A string identifier to distinguish the motor from others
	 *
	 * @return The motors position value identifier
	 */
	public static ValueIdentifier<Double> positionIdentifier(Enum<?> motorId) {
		return ValueIdentifier.get("/motors/" + motorId.name() + "/position", 0.0);
	}
	
	/**
	 * Gets a value identifier for a motors velocity
	 *
	 * @param motorId A string identifier to distinguish the motor from others
	 *
	 * @return The motors velocity value identifier
	 */
	public static ValueIdentifier<Double> velocityIdentifier(Enum<?> motorId) {
		return ValueIdentifier.get("/motors/" + motorId.name() + "/velocity", 0.0);
	}
}
