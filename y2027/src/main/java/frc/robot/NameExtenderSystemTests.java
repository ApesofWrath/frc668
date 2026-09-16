package frc.robot;

import frc.framework.testengine.Testable;
import frc.framework.testengine.UnitTest;

/**
 * Tests for the {@link NameExtenderSystem}
 */
public class NameExtenderSystemTests extends Testable {
	/**
	 * @return A test with the name Ansh
	 */
	@UnitTest(
		name = "Name extension with 'Ansh'"
	)
	public static NameExtenderSystemTests ansh() {
		return new NameExtenderSystemTests("Ansh");
	}
	
	/**
	 * @return A test with the name Balthalthemax
	 */
	@UnitTest(
		name = "Name extension with 'Balthalthemax'"
	)
	public static NameExtenderSystemTests balthalthemax() {
		return new NameExtenderSystemTests("Balthalthemax");
	}
	
	private final String name;
	
	/**
	 * Construct a test instance
	 *
	 * @param name The name to test with
	 */
	public NameExtenderSystemTests(String name) {
		systems.addSystem(new NameExtenderSystem());
		this.name = name;
	}
	
	@Override
	public void execute() {
		set(NameExtenderSystem.NAME_TO_EXTEND_VALUE, name);
		update();
		check.valueIs(HelloSpeakerSystem.NAME_VALUE, name + " " + name + "ington");
	}
}
