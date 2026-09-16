package frc.framework.testengine;

import io.github.classgraph.ClassGraph;
import io.github.classgraph.ClassInfo;
import io.github.classgraph.ScanResult;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;

public class Testing {
	/**
	 * @return The path to write InfoDeck JSON data to
	 */
	public static String getOutputDataPath() {
		return System.getenv("CODEINFO_OUTPUT");
	}
	
	/**
	 * @return Should we write InfoDeck JSON data?
	 */
	public static boolean isTestingEnvironment() {
		return System.getenv().containsKey("CODEINFO_OUTPUT");
	}
	
	/**
	 * Run the test suite and save to a file
	 */
	public static void runTests() {
		if (!isTestingEnvironment()) {
			throw new RuntimeException("runTests was invoked without a testing environment");
		}
		
		System.out.println("Running unit tests...");
		System.out.println("NOTE: Analysis data will be written to " + getOutputDataPath());
		
		CodeInfo info = new CodeInfo();
		
		ArrayList<InternalTestToRun> tests = scanForAllTests(info);
		
		for (InternalTestToRun test : tests) {
			test.run(info);
		}
		
		System.out.println(info);
	}
	
	/**
	 * Locate all the tests in the codebase
	 *
	 * @param info The code information to add potential lint errors to
	 *
	 * @return The tests that need to be run
	 */
	private static ArrayList<InternalTestToRun> scanForAllTests(CodeInfo info) {
		ArrayList<InternalTestToRun> tests = new ArrayList<>();
		
		try (ScanResult scanResult = new ClassGraph().acceptPackages("frc.robot", "frc.framework")
			.enableClassInfo()
			.enableMethodInfo()
			.enableAnnotationInfo()
			.scan()) {
			for (ClassInfo testClassInfo : scanResult.getClassesWithMethodAnnotation(UnitTest.class)) {
				Class<?> cls = testClassInfo.loadClass();
				
				for (Method method : cls.getDeclaredMethods()) {
					if (!method.isAnnotationPresent(UnitTest.class)) {
						continue;
					}
					
					if ((method.getModifiers() & Modifier.STATIC) == 0) {
						info.addLint(
							"The method " + method.getName() + " on the class " + cls
								.getName() + " has the @CreatesTest annotation, but isn't static."
						);
						continue;
					}
					
					if (!Testable.class.isAssignableFrom(method.getReturnType())) {
						info.addLint(
							"The method " + method.getName() + " on the class " + cls
								.getName() + " has the @CreatesTest annotation, doesn't return a " + Testable.class
									.getName() + "."
						);
						continue;
					}
					
					InternalTestToRun test = new InternalTestToRun();
					
					test.method = method;
					
					tests.add(test);
				}
			}
		}
		return tests;
	}
}
