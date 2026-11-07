package frc.framework.execution;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.logging.DataLogUtils;
import frc.framework.logging.LogFrame;
import frc.framework.logging.PackHelper;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.Priority;

import java.util.Arrays;
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
	 * @param time  The current time, used for caching
	 * @param frame The logging frame, used to capture the internal computations of the robot
	 * @param plan  The plan to execute
	 */
	public void execute(long time, LogFrame frame, ExecutionPlan plan) {
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
				previousResult.storeToLog(node.getId(), frame);
				continue;
			}
			
			NodeCachableResult result = new NodeCachableResult(time);
			NodeUpdateHelper helper = new NodeUpdateHelper(node, plan.nodeToInformation.get(node), this, result);
			
			node.update(helper);
			
			result.apply(this, time);
			result.storeToLog(node.getId(), frame);
			
			cachedResults.put(node, result);
		}
		
		for (String key : values.keySet()) {
			TimestampedValue<?> value = values.get(key);
			
			frame.set("/values/" + key, value.getValue());
			frame.set("/values/" + key + "/timestamp", value.getTimestamp());
			frame.set("/values/" + key + "/priority", value.getPriority().name());
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
			values.put(id, new TimestampedValue<>(null, Integer.MIN_VALUE, Priority.Default, this));
		}
		
		return (TimestampedValue<T>) values.get(id);
	}
	
	/**
	 * Write all value data to NetworkTables for debugging purposes.
	 *
	 * @param frame The frame to publish to NetworkTables
	 */
	public void publishValuesToNetworkTables(LogFrame frame) {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		
		PackHelper helper = new PackHelper();
		
		frame.data.forEach((key, value) -> {
			if (DataLogUtils.getTypeId(value) == null) {
				helper.unpackFields(key, value, false);
			} else {
				helper.setValue(key, value);
			}
		});
		
		helper.getData().forEach((key, value) -> {
			NetworkTableEntry entry = nt.getEntry(DataLogUtils.joinKey(key));
			
			if (value instanceof String val) {
				entry.setString(val);
			} else if (value instanceof String[] val) {
				entry.setStringArray(val);
			} else if (value instanceof Integer val) {
				entry.setInteger(val);
			} else if (value instanceof Integer[] val) {
				entry.setIntegerArray(Arrays.stream(val).mapToLong(it -> it).toArray());
			} else if (value instanceof Long val) {
				entry.setInteger(val);
			} else if (value instanceof Long[] val) {
				entry.setIntegerArray(val);
			} else if (value instanceof Boolean val) {
				entry.setBoolean(val);
			} else if (value instanceof Boolean[] val) {
				entry.setBooleanArray(val);
			} else if (value instanceof Float val) {
				entry.setFloat(val);
			} else if (value instanceof Float[] val) {
				entry.setFloatArray(val);
			} else if (value instanceof Double val) {
				entry.setDouble(val);
			} else if (value instanceof Double[] val) {
				entry.setDoubleArray(val);
			} else {
				throw new UnsupportedOperationException("Cannot publish NT value of " + value);
			}
		});
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
