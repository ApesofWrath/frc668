package frc.framework.testengine;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.framework.systems.SystemsManager;
import frc.framework.systems.ValueIdentifier;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static java.text.MessageFormat.format;

public class SystemsAssertions {
	private final SystemsManager manager;
	private final TestInfo testInfo;
	
	/**
	 * Creates an assertion helper
	 *
	 * @param manager  The SystemsManager reference
	 * @param testInfo A reference to the test information to add check information to
	 */
	public SystemsAssertions(SystemsManager manager, TestInfo testInfo) {
		this.manager = manager;
		this.testInfo = testInfo;
	}
	
	/**
	 * Claim that condition should be true
	 *
	 * @param condition The assertion that should be true
	 * @param message   What the assertion represents
	 */
	public void check(boolean condition, String message) {
		TestInfo.TestCheck check = new TestInfo.TestCheck();
		check.succeeded = condition;
		check.message = message;
		
		testInfo.checks.add(check);
	}
	
	/**
	 * Claim that an assertion failed
	 *
	 * @param message What the assertion represents
	 * @param reason  Why the assertion failed
	 */
	public void fail(String message, String reason) {
		TestInfo.TestCheck check = new TestInfo.TestCheck();
		check.succeeded = false;
		check.failureReason = reason;
		check.message = message;
		
		testInfo.checks.add(check);
	}
	
	/**
	 * Assert that A is approximately B
	 *
	 * @param a         The first value
	 * @param b         The second value
	 * @param threshold The maximum difference between the two values
	 * @param message   The message describing this assertion
	 */
	public void isApproximately(double a, double b, double threshold, String message) {
		check(Math.abs(a - b) < threshold, message);
	}
	
	/**
	 * Assert that A is approximately B
	 *
	 * @param a         The first value
	 * @param b         The second value
	 * @param threshold The maximum difference between the two values
	 * @param message   The message describing this assertion
	 */
	public void isApproximately(float a, float b, float threshold, String message) {
		check(Math.abs(a - b) < threshold, message);
	}
	
	/**
	 * Assert that A is approximately B
	 *
	 * @param a                  The first pose
	 * @param b                  The second pose
	 * @param distance           The maximum translational error between the two poses
	 * @param maxAngularDistance The maximum angular error between the two poses
	 * @param message            The message describing this assertion
	 */
	public void isApproximately(Pose2d a, Pose2d b, Distance distance, Angle maxAngularDistance, String message) {
		isApproximately(a, b, distance.in(Meters), maxAngularDistance.in(Degrees), message);
	}
	
	/**
	 * Assert that A is approximately B
	 *
	 * @param a                  The first pose
	 * @param b                  The second pose
	 * @param distance           The maximum translational error between the two poses (in meters)
	 * @param maxAngularDistance The maximum angular error between the two poses (in degrees)
	 * @param message            The message describing this assertion
	 */
	public void isApproximately(Pose2d a, Pose2d b, double distance, double maxAngularDistance, String message) {
		check(a.getTranslation().getDistance(b.getTranslation()) < distance, message);
		check(Math.abs(a.getRotation().getDegrees() - b.getRotation().getDegrees()) < maxAngularDistance, message);
	}
	
	/**
	 * Assert that A is approximately B
	 *
	 * @param a         The first translation
	 * @param b         The second translation
	 * @param threshold The maximum distance between the two translation
	 * @param message   The message describing this assertion
	 */
	public void isApproximately(Translation2d a, Translation2d b, Distance threshold, String message) {
		isApproximately(a, b, threshold.in(Meters), message);
	}
	
	/**
	 * Assert that A is approximately B
	 *
	 * @param a         The first translation
	 * @param b         The second translation
	 * @param threshold The maximum distance (in meters) between the two translation
	 * @param message   The message describing this assertion
	 */
	public void isApproximately(Translation2d a, Translation2d b, double threshold, String message) {
		check(a.getDistance(b) < threshold, message);
	}
	
	/**
	 * Claim that an assertion succeeded
	 *
	 * @param message What the assertion represents
	 */
	public void succeed(String message) {
		TestInfo.TestCheck check = new TestInfo.TestCheck();
		check.succeeded = true;
		check.message = message;
		
		testInfo.checks.add(check);
	}
	
	/**
	 * Assert that a value in a slot is the same as expectedValue
	 *
	 * @param ident         The slot identifier to check
	 * @param expectedValue The desired value in the slot
	 * @param <T>           The type of the value in the slot
	 */
	public <T> void valueIs(ValueIdentifier<T> ident, T expectedValue) {
		String message = format("{0} is {1}", ident.getId(), expectedValue.toString());
		
		valueIs(ident, expectedValue, message);
	}
	
	/**
	 * Assert that a value in a slot is the same as expectedValue
	 *
	 * @param ident         The slot identifier to check
	 * @param expectedValue The desired value in the slot
	 * @param message       The message to show upon failure
	 * @param <T>           The type of the value in the slot
	 */
	private <T> void valueIs(ValueIdentifier<T> ident, T expectedValue, String message) {
		T actualValue = manager.getValue(ident);
		
		if (actualValue.equals(expectedValue)) {
			succeed(message);
		} else {
			fail(message, format("{0} is {1} instead", ident.getId(), actualValue.toString()));
		}
	}
}
