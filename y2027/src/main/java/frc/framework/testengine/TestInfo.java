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
		
		/**
		 * Converts this check into a serializable protobuf message
		 *
		 * @return The protobuf message
		 */
		public UnitTests.CheckData toProtobuf() {
			return UnitTests.CheckData.newBuilder()
				.setFailureReason(failureReason)
				.setName(message)
				.setSucceeded(succeeded)
				.build();
		}
		
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
	
	/**
	 * The class in which this test is contained
	 */
	public String containingClass = "";
	
	/**
	 * Converts the test information to a serializable protobuf
	 *
	 * @return The serialized protobuf
	 */
	public UnitTests.UnitTestData toProtobuf() {
		return UnitTests.UnitTestData.newBuilder()
			.setName(name)
			.setContainingClass(containingClass)
			.addAllChecks(checks.stream().map(it -> it.toProtobuf()).toList())
			.build();
	}
	
	@Override
	public String toString() {
		return "TestInfo{" + "checks=" + checks + ", errors=" + errors + ", name='" + name + '\'' + '}';
	}
}
