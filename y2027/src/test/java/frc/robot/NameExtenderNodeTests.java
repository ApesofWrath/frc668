package frc.robot;

import frc.framework.RobotTest;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class NameExtenderNodeTests extends RobotTest {
	@DisplayName(
		"Name Extension of '123'"
	) @Test
	public void nameExtensionTest() {
		addNode(new NameExtenderNode());
		
		set(NameExtenderNode.NAME_TO_EXTEND_VALUE, "123");
		advance();
		assertEquals("123 123ington", get(HelloSpeakerNode.NAME_VALUE).orNull(10));
	}
}
