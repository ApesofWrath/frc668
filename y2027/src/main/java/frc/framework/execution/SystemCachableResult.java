package frc.framework.execution;

import frc.framework.logging.LogFrame;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map.Entry;

/**
 * Represents the outputs of a system at a given type
 */
public class SystemCachableResult {
	/**
	 * The time at which the system was evaluated
	 */
	public long timeCreated;
	/**
	 * A mapping from string identifiers to the values outputted
	 */
	public HashMap<String, Object> values = new HashMap<>();
	/**
	 * A mapping from string identifiers to the priority of values outputted
	 */
	public HashMap<String, Integer> priorities = new HashMap<>();
	/**
	 * A mapping from string identifiers to values read during the evaluation of a system
	 */
	public HashMap<String, Object> accessedValues = new HashMap<>();
	
	/**
	 * Create a result with a given time value
	 *
	 * @param time The time value of the result
	 */
	public SystemCachableResult(long time) {
		this.timeCreated = time;
	}
	
	/**
	 * Replay/apply the computed values to an {@link ExecutionManager}
	 *
	 * @param manager The {@link ExecutionManager} that values should be outputted to
	 */
	public void apply(ExecutionManager manager) {
		for (String key : values.keySet()) {
			Object value = values.get(key);
			int priority = priorities.get(key);
			
			manager.trySetValue(key, value, priority);
		}
	}
	
	/**
	 * Returns true if all the values read by the system during computation match the current values
	 *
	 * @param manager The ExecutionManager to read the values from
	 *
	 * @return A boolean indicating if values are the same as before
	 */
	public boolean doInputsMatchWithManager(ExecutionManager manager) {
		for (Entry<String, Object> entry : accessedValues.entrySet()) {
			Object currentValue = manager.getValue(entry.getKey());
			
			if (!currentValue.equals(entry.getValue())) {
				return false;
			}
		}
		
		return true;
	}
	
	/**
	 * Get a value while storing the current accessed value for cache invalidation purposes
	 *
	 * @param executor The ExecutionManager to read the values from
	 * @param key      The string identifier of the slot to read values from
	 *
	 * @return The value contained within the slot with the given key
	 */
	public Object get(ExecutionManager executor, String key) {
		Object value = executor.getValue(key);
		accessedValues.put(key, value);
		return value;
	}
	
	/**
	 * Returns true if this is younger than a given age
	 *
	 * @param time                 The current time in milliseconds
	 * @param millisecondThreshold The maximum age, after which this method will return false
	 *
	 * @return Is the result fresh
	 */
	public boolean isFresh(int time, int millisecondThreshold) {
		return (time - timeCreated) < millisecondThreshold;
	}
	
	/**
	 * Record an output to this result, to be written later.
	 *
	 * @param key      The string identifier of the slot to be written to
	 * @param value    The value to write to the slot
	 * @param priority The priority with which the value should be written
	 */
	public void set(String key, Object value, int priority) {
		values.put(key, value);
		priorities.put(key, priority);
	}
	
	public void storeToLog(String id, LogFrame frame) {
		ArrayList<String> outputs = new ArrayList<>(values.keySet());
		
		frame.set("/system_outputs/" + id + "/timestamp", timeCreated);
		frame.set("/system_outputs/" + id + "/outputs", outputs.toArray());
		
		for (String key : values.keySet()) {
			Object value = values.get(key);
			int priority = priorities.get(key);
			frame.set("/system_outputs/" + id + "/outputs/" + key + "/value", value);
			frame.set("/system_outputs/" + id + "/outputs/" + key + "/priority", priority);
		}
	}
}
