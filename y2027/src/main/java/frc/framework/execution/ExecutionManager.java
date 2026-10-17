package frc.framework.execution;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.Priority;

import java.util.HashMap;

/**
 * Manages the execution of and caching of nodes
 */
public class ExecutionManager {
	private final HashMap<String, TimestampedValue<?>> values = new HashMap<>();
	private final HashMap<Node, NodeCachableResult> cachedResults = new HashMap<>();
	private long timestamp = 0;
	
	/**
	 * Evaluate the nodes
	 *
	 * @param time The current time, used for caching
	 * @param plan The plan to execute
	 */
	public void execute(long time, ExecutionPlan plan) {
		timestamp = time;
		values.put(
			RobotInformation.TIMESTAMP_VALUE.getId(),
			new TimestampedValue<>(time, time, Priority.Default, this)
		);
		
		// run plan
		for (Node node : plan.nodeExecutionOrder) {
			NodeCachableResult previousResult = cachedResults.getOrDefault(node, null);
			NodeInformation info = plan.nodeToInformation.get(node);
			
			if (previousResult != null && info.cacheStrategy.isCacheValid(time, this, previousResult)) {
				previousResult.apply(this, time);
				continue;
			}
			
			NodeCachableResult result = new NodeCachableResult(time);
			NodeUpdateHelper helper = new NodeUpdateHelper(node, plan.nodeToInformation.get(node), this, result);
			
			node.update(helper);
			
			result.apply(this, time);
			
			cachedResults.put(node, result);
		}
	}
	
	/**
	 * @return The current time in milliseconds
	 */
	public long getTimestamp() {
		return timestamp;
	}
	
	/**
	 * Get the current value in a given slot
	 *
	 * @param id  The string identifier of the slot
	 * @param <T> The type of data contained in the slot
	 *
	 * @return The value contained within the slot
	 */
	@SuppressWarnings(
		"unchecked"
	)
	public <T> TimestampedValue<T> getValue(String id) {
		if (!values.containsKey(id)) {
			values.put(id, new TimestampedValue<>(null, 0, Priority.Default, this));
		}
		
		return (TimestampedValue<T>) values.get(id);
	}
	
	/**
	 * Write all value data to NetworkTables for debugging purposes.
	 */
	public void publishValuesToNetworkTables() {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		
		for (java.util.Map.Entry<String, TimestampedValue<?>> pair : values.entrySet()) {
			String key = pair.getKey();
			
			if (key.startsWith("/")) {
				key = key.substring(1);
			}
			
			key = "Systems/" + key;
			
			Object value = pair.getValue().getValue();
			
			if (value instanceof Integer integerValue) {
				nt.getEntry(key).setInteger(integerValue);
			} else if (value instanceof Long longValue) {
				nt.getEntry(key).setInteger(longValue);
			} else if (value instanceof String stringValue) {
				nt.getEntry(key).setString(stringValue);
			} else if (value instanceof Boolean booleanValue) {
				nt.getEntry(key).setBoolean(booleanValue);
			}
		}
	}
	
	/**
	 * Try to assign a timestamped value to a given slot, not updating it if the time or the priority is too low
	 *
	 * @param key   The string identifer of the slot to assign to
	 * @param value The value to try to set
	 */
	public void trySetValue(String key, TimestampedValue<?> value) {
		if (value.takesPrecedenceOver(getValue(key))) {
			values.put(key, value);
		}
	}
}
