package frc.framework.execution;

import frc.framework.systems.Priority;

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
	public HashMap<String, Priority> priorities = new HashMap<>();
	/**
	 * A mapping from string identifiers to values read during the evaluation of a system
	 */
	public HashMap<String, TimestampedValue<?>> accessedValues = new HashMap<>();
	
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
	 * @param time    The time to attempt to apply the values using
	 */
	public void apply(ExecutionManager manager, long time) {
		for (String key : values.keySet()) {
			Object value = values.get(key);
			Priority priority = priorities.get(key);
			
			manager.trySetValue(key, new TimestampedValue<>(value, time, priority, manager));
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
		for (Entry<String, TimestampedValue<?>> entry : accessedValues.entrySet()) {
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
	public TimestampedValue<?> get(ExecutionManager executor, String key) {
		TimestampedValue<?> value = executor.getValue(key);
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
	public void set(String key, Object value, Priority priority) {
		values.put(key, value);
		priorities.put(key, priority);
	}
}
