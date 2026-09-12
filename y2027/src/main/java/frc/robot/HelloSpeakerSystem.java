package frc.robot;

import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.framework.systems.ValueIdentifier;

public class HelloSpeakerSystem implements System {
	public static ValueIdentifier<String> NAME_VALUE =
			new ValueIdentifier<String>("/hello/name", "World");

	@Override
	public void configure(SystemInformation information) {
		information.recievesInput(NAME_VALUE);
	}

	@Override
	public void update(SystemUpdateHelper update) {
		java.lang.System.out.println("Hello, " + update.getValue(NAME_VALUE));
	}
}
