package frc.robot;

import frc.framework.cache.MemoizationCacheStrategy;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.framework.systems.ValueIdentifier;

/**
 * Extends a name {@link NameExtenderSystem#NAME_TO_EXTEND_VALUE} to Name Nameington, and outputs to
 * {@link HelloSpeakerSystem#NAME_VALUE}
 */
public class NameExtenderSystem implements System {
	/**
	 * The name to extend
	 */
	public static ValueIdentifier<String> NAME_TO_EXTEND_VALUE = ValueIdentifier.get(
		"/name_extender/name_to_extend",
		"World"
	);
	
	@Override
	public void configure(SystemInformation information) {
		information.cacheStrategy = new MemoizationCacheStrategy();
		information.recievesInput(NAME_TO_EXTEND_VALUE);
		information.createsOutput(HelloSpeakerSystem.NAME_VALUE);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		java.lang.System.out.println("extending name");
		String name = update.getValue(NAME_TO_EXTEND_VALUE);
		update.setValue(HelloSpeakerSystem.NAME_VALUE, name + " " + name + "ington");
	}
}
