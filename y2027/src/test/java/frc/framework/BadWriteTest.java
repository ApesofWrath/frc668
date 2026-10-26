package frc.framework;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.robot.TestNodeIdentifiers;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class BadWriteNode implements Node {
	@Override
	public void configure(NodeInformation information) {
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		update.setValue(TestNodeIdentifiers.alpha, 1);
	}
}

public class BadWriteTest extends RobotTest {
	@Test
	@DisplayName(
		"Bad reads throw"
	)
	public void badRead() {
		addNode(new BadWriteNode());
		RuntimeException exception = assertThrows(RuntimeException.class, this::advance);
		assertNull(exception.getCause());
		assertEquals(
			"The node " + BadWriteNode.class.getName() + " tried to set the value of " + TestNodeIdentifiers.alpha
				.getId() + ", but it never specified ahead of time that it sets it, this is throwing an error as it could cause unintended logic bugs.",
			exception.getMessage()
		);
	}
}
