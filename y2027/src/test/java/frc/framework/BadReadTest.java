package frc.framework;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.robot.TestNodeIdentifiers;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class BadReadNode implements Node {
	@Override
	public void configure(NodeInformation information) {
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		update.getValue(TestNodeIdentifiers.alpha);
	}
}

public class BadReadTest extends RobotTest {
	@Test
	@DisplayName(
		"Bad reads throw"
	)
	public void badRead() {
		addNode(new BadReadNode());
		RuntimeException exception = assertThrows(RuntimeException.class, this::advance);
		assertNull(exception.getCause());
		assertEquals(
			"The node " + BadReadNode.class.getName() + " tried to get the value of " + TestNodeIdentifiers.alpha
				.getId() + ", but it never specified ahead of time that it needs it, thus, we cannot be sure that the value has been set.",
			exception.getMessage()
		);
	}
}
