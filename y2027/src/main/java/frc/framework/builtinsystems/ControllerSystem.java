package frc.framework.builtinsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.systems.Priority;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

public class ControllerSystem implements System {
	public XboxController xbox = new XboxController(0);
	public Joystick joystick = new Joystick(0);

	@Override
	public void configure(SystemInformation information) {
		information.createsOutput(UserInputInformation.CONTROLLER_INPUT);
	}

	@Override
	public void update(SystemUpdateHelper update) {
		ControllerState state = new ControllerState();

		if (joystick.isConnected()) {
			state.setAxis(ControllerState.Axis.LeftThumbstickX, joystick.getX());
			state.setAxis(ControllerState.Axis.LeftThumbstickY, joystick.getY());
		}

		if (xbox.isConnected()) {
			state.setAxis(ControllerState.Axis.LeftThumbstickX, xbox.getLeftX());
			state.setAxis(ControllerState.Axis.LeftThumbstickY, xbox.getLeftY());

			state.setAxis(ControllerState.Axis.RightThumbstickX, xbox.getRightX());
			state.setAxis(ControllerState.Axis.RightThumbstickY, xbox.getRightY());

			state.setButtonState(ControllerState.Button.A, xbox.getAButton());
			state.setButtonState(ControllerState.Button.B, xbox.getBButton());
			state.setButtonState(ControllerState.Button.X, xbox.getXButton());
			state.setButtonState(ControllerState.Button.Y, xbox.getYButton());

			state.setButtonState(ControllerState.Button.L1, xbox.getLeftBumperButton());
			state.setButtonState(ControllerState.Button.R1, xbox.getRightBumperButton());

			state.setButtonState(
					ControllerState.Button.L2,
					xbox.getLeftTriggerAxis() > 0.5
			);
			state.setButtonState(
					ControllerState.Button.R2,
					xbox.getRightTriggerAxis() > 0.5
			);

			state.setButtonState(ControllerState.Button.L3, xbox.getLeftStickButton());
			state.setButtonState(ControllerState.Button.R3, xbox.getRightStickButton());

			state.setButtonState(ControllerState.Button.Start, xbox.getStartButton());
		}

		update.setValue(
				UserInputInformation.CONTROLLER_INPUT,
				state,
				Priority.DriverInput
		);
	}
}
