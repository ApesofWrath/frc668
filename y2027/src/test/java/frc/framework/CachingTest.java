package frc.framework;

import frc.framework.cache.MemoizationCacheStrategy;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class CachingTest extends RobotTest {
	@Test
	@DisplayName(
		"Expiry triggers invalidation"
	)
	public void ensureExpiryTriggersInvaldation() {
		NumberDoublerNode doubler = new NumberDoublerNode();
		addNode(doubler);
		
		set(NumberDoublerNode.NUMBER_TO_DOUBLE, 1.0);
		advance(Seconds.of(1));
		assertEquals(2, doubler.runs);
		assertEquals(0, get(NumberDoublerNode.DOUBLED_NUMBER).orNull());
	}
	
	@Test
	@DisplayName(
		"Correct number of invalidations"
	)
	public void ensureUpdating() {
		NumberDoublerNode doubler = new NumberDoublerNode();
		addNode(doubler);
		
		set(NumberDoublerNode.NUMBER_TO_DOUBLE, 1.0);
		advance();
		assertEquals(1, doubler.runs);
		assertEquals(2, get(NumberDoublerNode.DOUBLED_NUMBER).orNull());
		
		set(NumberDoublerNode.NUMBER_TO_DOUBLE, 2.0);
		advance();
		assertEquals(2, doubler.runs);
		assertEquals(4, get(NumberDoublerNode.DOUBLED_NUMBER).orNull());
	}
}

class NumberDoublerNode implements Node {
	public static final ValueIdentifier<Double> NUMBER_TO_DOUBLE = ValueIdentifier.get(
		"/test/number_to_double",
		Double.class
	);
	public static final ValueIdentifier<Double> DOUBLED_NUMBER = ValueIdentifier.get(
		"/test/doubled_number",
		Double.class
	);
	
	public int runs = 0;
	
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(NUMBER_TO_DOUBLE);
		information.createsOutput(DOUBLED_NUMBER);
		information.cacheStrategy = new MemoizationCacheStrategy();
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		runs++;
		
		double number = update.getValue(NUMBER_TO_DOUBLE).orDefault(0.0, 500);
		
		update.setValue(DOUBLED_NUMBER, number * 2);
	}
}
