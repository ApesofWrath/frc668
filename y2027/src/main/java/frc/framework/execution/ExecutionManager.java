package frc.framework.execution;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.systems.*;
import frc.framework.systems.System;

import java.util.HashMap;
import java.util.Map;

/**
 * Manages the execution of and caching of systems
 */
public class ExecutionManager {
	private final HashMap<String, Object> values = new HashMap<>();
	private final HashMap<String, Integer> valuePriorities = new HashMap<>();
	private final HashMap<System, SystemCachableResult> cachedResults = new HashMap<>();
	/**
	 * The plan to use for execution, containing the order that systems should be evaluated
	 */
	public ExecutionPlan plan;
	
	/**
	 * Evaluate the systems
	 *
	 * @param time       The current time, used for caching
	 * @param testInputs The inputs assigned by unit tests
	 */
	public void execute(long time, HashMap<String, Object> testInputs) {
		// reset values
		values.clear();
		valuePriorities.clear();
		
		for (Map.Entry<String, ?> data : testInputs.entrySet()) {
			values.put(data.getKey(), data.getValue());
			valuePriorities.put(data.getKey(), Priority.TestingData.value);
		}
		
		values.put(RobotInformation.TIMESTAMP_VALUE.getId(), time);
		// run plan
		for (System system : plan.systemExecutionOrder) {
			SystemCachableResult previousResult = cachedResults.getOrDefault(system, null);
			SystemInformation info = plan.systemToInformation.get(system);
			
			if (previousResult != null && info.cacheStrategy.isCacheValid(time, this, previousResult)) {
				previousResult.apply(this);
				continue;
			}
			
			SystemCachableResult result = new SystemCachableResult(time);
			SystemUpdateHelper helper = new SystemUpdateHelper(
				system,
				plan.systemToInformation.get(system),
				this,
				result
			);
			
			system.update(helper);
			
			result.apply(this);
			
			cachedResults.put(system, result);
		}
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
	public <T> T getValue(String id) {
		if (!values.containsKey(id)) {
			values.put(id, ValueIdentifier.get(id).getDefaultValue());
		}
		
		return (T) values.get(id);
	}
	
	/**
	 * Write all value data to NetworkTables for debugging purposes.
	 */
	public void publishValuesToNetworkTables() {
		NetworkTableInstance nt = NetworkTableInstance.getDefault();
		
		for (java.util.Map.Entry<String, Object> pair : values.entrySet()) {
			String key = pair.getKey();
			
			if (key.startsWith("/")) {
				key = key.substring(1);
			}
			
			key = "Systems/" + key;
			
			Object value = pair.getValue();
			
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
	 * Try to assign a value to a given slot, with a certain priority
	 *
	 * @param key      The string identifer of the slot to assign to
	 * @param value    The value to put in the slot
	 * @param priority The priority to write to the slot, typically gotten by accessing
	 *                 {@link frc.framework.systems.Priority#value}
	 */
	public void trySetValue(String key, Object value, int priority) {
		int currentPriority = valuePriorities.getOrDefault(key, Integer.MIN_VALUE);
		
		if (priority > currentPriority) {
			values.put(key, value);
			valuePriorities.put(key, priority);
		}
	}
}
