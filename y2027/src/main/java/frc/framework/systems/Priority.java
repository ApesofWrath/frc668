package frc.framework.systems;

/**
 * An enumeration where {@link #value} is a number indicating the priority that outputted values should have, higher
 * beats lower.
 */
public enum Priority {
	/**
	 * For data written by unit tests
	 */
	TestingData(6),
	/**
	 * For values that are safety critical, and should beat all other values in real world scenarios
	 */
	Safety(5),
	/**
	 * For values generated in the autonomous period
	 */
	Autonomous(4),
	/**
	 * For values that aid the driver, and overwrite the driver's input
	 */
	DriverAssistanceOverridesDriver(3),
	/**
	 * Input directly triggered by the driver
	 */
	DriverInput(2),
	/**
	 * For values that aid the driver, taking lower priority than the driver's input
	 */
	DriverAssistance(1),
	/**
	 * The default priority value, should be avoided if possible
	 */
	Default(0);
	
	/**
	 * A numeric representation of priority, higher beats lower
	 */
	public final int value;
	
	Priority(int value) {
		this.value = value;
	}
}
