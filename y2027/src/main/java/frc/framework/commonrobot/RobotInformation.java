package frc.framework.commonrobot;

import frc.framework.OpMode;
import frc.framework.systems.ValueIdentifier;

/**
 * Contains various ValueIdentifies relating to HAL information.
 */
public class RobotInformation {
	/**
	 * The current robot operating mode
	 */
	public static ValueIdentifier<OpMode> OPMODE_VALUE = ValueIdentifier.get("/hal/opmode", OpMode.Disabled);
	/**
	 * The current timestamp
	 * <p>
	 * This is a delicate API, as although this is a UNIX timestamp in the real world, in tests it will typically tick
	 * up from 0
	 */
	public static ValueIdentifier<Long> TIMESTAMP_VALUE = ValueIdentifier.get("/hal/timestamp", 0L);
}
