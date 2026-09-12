package frc.robot;

import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

public class AnshSystem implements System {
	@Override
	public void configure(SystemInformation information) {
		information.createsOutput(NameExtenderSystem.NAME_TO_EXTEND_VALUE);
	}

	@Override
	public void update(SystemUpdateHelper update) {
		update.setValue(NameExtenderSystem.NAME_TO_EXTEND_VALUE, "Ansh");
	}
}
