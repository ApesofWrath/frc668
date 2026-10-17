package frc.framework.commonrobot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

/**
 * Determines how a motor should accelerate
 *
 * @param type  The value to control
 * @param value The controlled value
 */
public record MotorControl(MotorControlType type, double value) {
	/**
	 * Creates a motor control to attempt to accelerate to a given velocity
	 *
	 * @param radiansPerSecond The velocity to accelerate to
	 *
	 * @return The motor control
	 */
	public static MotorControl velocity(double radiansPerSecond) {
		return new MotorControl(MotorControlType.Velocity, radiansPerSecond);
	}
	
	/**
	 * Creates a motor control to attempt to accelerate to a given velocity
	 *
	 * @param velocity The velocity to accelerate to
	 *
	 * @return The motor control
	 */
	public static MotorControl velocity(AngularVelocity velocity) {
		return new MotorControl(MotorControlType.Velocity, velocity.in(RadiansPerSecond));
	}
	
	
	/**
	 * Creates a motor control to attempt to reach a given position
	 *
	 * @param radians The position (in radians) to go towards
	 *
	 * @return The motor control
	 */
	public static MotorControl position(double radians) {
		return new MotorControl(MotorControlType.Position, radians);
	}
	
	/**
	 * Creates a motor control to attempt to reach a given position
	 *
	 * @param angle The position to go towards
	 *
	 * @return The motor control
	 */
	public static MotorControl position(Angle angle) {
		return new MotorControl(MotorControlType.Position, angle.in(Radians));
	}
	
	
	/**
	 * Creates a motor control to apply a given voltage
	 *
	 * @param volts The voltage to use
	 *
	 * @return The motor control
	 */
	public static MotorControl voltage(double volts) {
		return new MotorControl(MotorControlType.Voltage, volts);
	}
	
	/**
	 * Creates a motor control to apply a given voltage
	 *
	 * @param voltage The voltage to use
	 *
	 * @return The motor control
	 */
	public static MotorControl voltage(Voltage voltage) {
		return new MotorControl(MotorControlType.Voltage, voltage.in(Volts));
	}
}
