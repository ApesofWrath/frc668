package frc.framework;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.UndeclaredWriteException;
import frc.robot.TestNodeIdentifiers;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

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
		"Bad writes throw"
	)
	public void badWrite() {
		BadWriteNode node = new BadWriteNode();
		addNode(node);
		
		UndeclaredWriteException exception = assertThrows(UndeclaredWriteException.class, this::advance);
		assertEquals(node, exception.getNode());
		assertEquals(TestNodeIdentifiers.alpha, exception.getIdentifier());
	}
}
