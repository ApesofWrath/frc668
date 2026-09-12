package frc.framework.builtinsystems;

import frc.framework.OpMode;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.systems.Priority;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

public class HALRobotInformationSystem implements System {
	public OpMode opMode;

	@Override
	public void configure(SystemInformation information) {
		information.createsOutput(RobotInformation.OPMODE_VALUE);
	}

	@Override
	public void update(SystemUpdateHelper update) {
		update.setValue(RobotInformation.OPMODE_VALUE, opMode, Priority.Safety);
	}
}
