package frc.framework.testengine;

import java.util.ArrayList;

/**
 * Outputted data pertaining to the entire codebase
 */
public class CodeInfo {
	private final ArrayList<String> lints = new ArrayList<>();
	private final ArrayList<TestInfo> tests = new ArrayList<>();
	
	/**
	 * Note that there is a code quality issue to be surfaced
	 *
	 * @param issue The issue with the code to surface
	 */
	public void addLint(String issue) {
		if (lints.contains(issue)) {
			return;
		}
		
		lints.add(issue);
	}
	
	/**
	 * Add the data pertaining to a given test
	 *
	 * @param testInfo The data about the test
	 */
	public void addTest(TestInfo testInfo) {
		tests.add(testInfo);
	}
	
	/**
	 * @return All the detected code quality issues
	 */
	public ArrayList<String> getLints() {
		return lints;
	}
	
	/**
	 * @return All the test data
	 */
	public ArrayList<TestInfo> getTests() {
		return tests;
	}
	
	@Override
	public String toString() {
		return "CodeInfo{" + "lints=" + lints + ", tests=" + tests + '}';
	}
}
