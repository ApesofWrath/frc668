package frc.robot;

import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.framework.systems.ValueIdentifier;

/**
 * Prints a hello message directed towards {@link HelloSpeakerSystem#NAME_VALUE} to the console
 */
public class HelloSpeakerSystem implements System {
	/**
	 * The name to say hi to
	 */
	public static ValueIdentifier<String> NAME_VALUE = ValueIdentifier.get("/hello/name", "World");
	
	@Override
	public void configure(SystemInformation information) {
		information.recievesInput(NAME_VALUE);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
	}
}
