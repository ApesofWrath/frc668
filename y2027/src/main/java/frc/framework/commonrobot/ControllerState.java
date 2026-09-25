package frc.framework.commonrobot;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Set;

/**
 * Represents the inputs of a controller at a given instant
 */
public class ControllerState {
	/**
	 * Identifies an analog input
	 */
	public enum Axis {
		/**
		 * The horizontal position of the left thumbstick
		 */
		LeftThumbstickX,
		/**
		 * The vertical position of the left thumbstick
		 */
		LeftThumbstickY,
		/**
		 * The horizontal position of the right thumbstick
		 */
		RightThumbstickX,
		/**
		 * The vertical position of the right thumbstick
		 */
		RightThumbstickY;
		
		/**
		 * Turn an ordinal into the representative axis ID
		 *
		 * @param ordinal The integer ordinal
		 *
		 * @return The axis ID
		 */
		public static Axis fromOrdinal(int ordinal) {
			return values()[ordinal];
		}
	}
	
	/**
	 * Identifies a digital input
	 */
	public enum Button {
		/**
		 * The Xbox A button
		 */
		A,
		/**
		 * The Xbox B button
		 */
		B,
		/**
		 * The Xbox X button
		 */
		X,
		/**
		 * The Xbox Y button
		 */
		Y,
		/**
		 * The left bumper button on a controller
		 */
		L1,
		/**
		 * The right bumper button on a controller
		 */
		R1,
		/**
		 * The left trigger on a controller
		 */
		L2,
		/**
		 * The right trigger on a controller
		 */
		R2,
		/**
		 * The depression of the left thumbstick
		 */
		L3,
		/**
		 * The depression of the right thumbstick
		 */
		R3,
		/**
		 * The Xbox button
		 */
		Start;
		
		/**
		 * Turn an ordinal into the representative button ID
		 *
		 * @param ordinal The integer ordinal
		 *
		 * @return The button ID
		 */
		public static Button fromOrdinal(int ordinal) {
			return values()[ordinal];
		}
	}
	
	private final HashMap<Axis, Double> axisData = new HashMap<>();
	
	private final HashMap<Button, Boolean> buttonStates = new HashMap<>();
	
	@Override
	public boolean equals(Object o) {
		if (o == null || getClass() != o.getClass()) { return false; }
		ControllerState that = (ControllerState) o;
		return Objects.equals(axisData, that.axisData) && Objects.equals(buttonStates, that.buttonStates);
	}
	
	/**
	 * Get the hashmap of axis ID to axis value
	 *
	 * @return The axis hashmap
	 */
	public Set<Map.Entry<Axis, Double>> getAllAxes() {
		return axisData.entrySet();
	}
	
	/**
	 * Get the hashmap of button ID to button state
	 *
	 * @return The button hashmap
	 */
	public Set<Map.Entry<Button, Boolean>> getAllButtons() {
		return buttonStates.entrySet();
	}
	
	/**
	 * Return the value of an analog input
	 *
	 * @param axis The analog input to read
	 *
	 * @return The current analog value
	 */
	public double getAxis(Axis axis) {
		return axisData.getOrDefault(axis, 0.0);
	}
	
	/**
	 * Return the value of a digital input
	 *
	 * @param button The digital input to read
	 *
	 * @return The current digital value
	 */
	public boolean getButtonState(Button button) {
		return buttonStates.getOrDefault(button, false);
	}
	
	@Override
	public int hashCode() {
		return Objects.hash(axisData, buttonStates);
	}
	
	/**
	 * Set to an analog value
	 *
	 * @param axis  The analog value to update
	 * @param value The value to update to
	 */
	public void setAxis(Axis axis, double value) {
		axisData.put(axis, value);
	}
	
	/**
	 * Set to a digital value
	 *
	 * @param button The digital value to update
	 * @param value  The value to update to
	 */
	public void setButtonState(Button button, boolean value) {
		buttonStates.put(button, value);
	}
	
	@Override
	public String toString() {
		return axisData + " " + buttonStates;
	}
}
