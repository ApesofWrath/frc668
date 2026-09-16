package frc.framework.testengine;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * A test inside the test runner to run
 */
public class InternalTestToRun {
	/**
	 * The method that returns a {@link Testable}
	 */
	public Method method;
	
	/**
	 * The display name of the test
	 */
	public String name;
	
	/**
	 * Run the test
	 *
	 * @param info The code information to append test information to
	 */
	public void run(CodeInfo info) {
		tryUpdateName();
		
		Testable testable;
		
		try {
			testable = (Testable) method.invoke(null);
		} catch (IllegalAccessException | InvocationTargetException e) {
			throw new RuntimeException(e);
		}
		
		TestInfo testInfo = testable.info;
		testInfo.name = name;
		
		testable.run();
		
		info.addTest(testInfo);
	}
	
	private void tryUpdateName() {
		name = method.getName();
		
		UnitTest annotation = method.getAnnotation(UnitTest.class);
		
		String annotationName = annotation.name();
		
		if (!annotationName.isEmpty()) {
			name = annotationName;
		}
	}
}
