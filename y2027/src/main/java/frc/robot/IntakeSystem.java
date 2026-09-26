package frc.robot;

import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.Motors;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

import static edu.wpi.first.units.Units.RPM;

/**
 * Manages intake rollers
 */
public class IntakeSystem implements System {
	@Override
	public void configure(SystemInformation information) {
		information.recievesInput(UserInputInformation.CONTROLLER_INPUT);
		information.createsOutput(Motors.controlIdentifier(MotorId.IntakeRoller));
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		ControllerState state = update.getValue(UserInputInformation.CONTROLLER_INPUT);
		
		if (state.getButtonState(ControllerState.Button.A)) {
			update.setValue(
				Motors.controlIdentifier(MotorId.IntakeRoller), // Set the intake motor control
				MotorControl.velocity(RPM.of(500))
			);
		}
	}
}
