package frc.framework.testengine;

import java.util.ArrayList;

/**
 * Information pertaining to a given test
 */
public class TestInfo {
	/**
	 * Information pertaining to an assertion in a test
	 */
	public static class TestCheck {
		/**
		 * Did the assertion succeed?
		 */
		public boolean succeeded = false;
		/**
		 * The name of the assertion
		 */
		public String message = "";
		/**
		 * More information about why {@link TestCheck#succeeded} may be false
		 */
		public String failureReason = "";
		
		@Override
		public String toString() {
			return "TestCheck{" + "succeeded=" + succeeded + ", message='" + message + '\'' + ", failureReason='" + failureReason + '\'' + '}';
		}
	}
	
	/**
	 * Information pertaining to the assertions in a test
	 */
	public ArrayList<TestCheck> checks = new ArrayList<>();
	/**
	 * Errors that arose during the test
	 */
	public ArrayList<String> errors = new ArrayList<>();
	
	/**
	 * The name of the test
	 */
	public String name = "";
	
	@Override
	public String toString() {
		return "TestInfo{" + "checks=" + checks + ", errors=" + errors + ", name='" + name + '\'' + '}';
	}
}
