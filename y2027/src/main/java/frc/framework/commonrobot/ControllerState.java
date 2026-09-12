package frc.framework.commonrobot;

import java.util.HashMap;

public class ControllerState {
	public enum Axis {
		LeftThumbstickX,
		LeftThumbstickY,
		RightThumbstickX,
		RightThumbstickY,
	}

	public enum Button {
		A,
		B,
		X,
		Y,
		L1,
		R1,
		L2,
		R2,
		L3,
		R3,
		Start,
	}

	private HashMap<Axis, Double> axisData = new HashMap<>();
	private HashMap<Button, Boolean> buttonStates = new HashMap<>();

    @Override
    public String toString() {
        return axisData.toString() + " " + buttonStates.toString();
    }

	public double getAxis(Axis axis) {
		return axisData.getOrDefault(axis, 0.0);
	}

	public void setAxis(Axis axis, double value) {
		axisData.put(axis, value);
	}

	public boolean getButtonState(Button button) {
		return buttonStates.getOrDefault(button, false);
	}

	public void setButtonState(Button button, boolean value) {
		buttonStates.put(button, value);
	}
}
