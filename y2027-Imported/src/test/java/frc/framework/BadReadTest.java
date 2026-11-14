package frc.framework;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.UndeclaredReadException;
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
		BadReadNode node = new BadReadNode();
		addNode(node);
		
		UndeclaredReadException exception = assertThrows(UndeclaredReadException.class, this::advance);
		assertNull(exception.getCause());
		assertEquals(node, exception.getNode());
		assertEquals(TestNodeIdentifiers.alpha, exception.getIdentifier());
	}
}
