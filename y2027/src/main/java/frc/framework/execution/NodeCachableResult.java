package frc.framework.execution;

import frc.framework.nodes.DataAccessNotifier;
import frc.framework.nodes.Priority;

import java.util.HashMap;
import java.util.HashSet;

/**
 * Represents the outputs of a node at a given type
 */
public class NodeCachableResult implements DataAccessNotifier {
	/**
	 * Represents a value access with a maximum age
	 *
	 * @param key      The key that was accessed
	 * @param value    The value at the moment
	 * @param maxAge   The maximum age specified during the access
	 * @param wasStale Was this value stale at the time of storage
	 */
	public record AccessedValue(String key, Object value, long maxAge, boolean wasStale) {
	}
	
	/**
	 * The time at which the node was evaluated
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
	 * A set of values read during the evaluation of this node
	 */
	public HashSet<AccessedValue> accessedValues = new HashSet<>();
	
	/**
	 * Create a result with a given time value
	 *
	 * @param time The time value of the result
	 */
	public NodeCachableResult(long time) {
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
			
			manager.trySetValue(key, new TimestampedValue<>(key, value, time, priority, manager));
		}
	}
	
	/**
	 * Returns true if all the values read by the node during computation match the current values
	 *
	 * @param manager The ExecutionManager to read the values from
	 *
	 * @return A boolean indicating if values are the same as before
	 */
	public boolean doInputsMatchWithManager(ExecutionManager manager) {
		for (AccessedValue accessedValue : accessedValues) {
			TimestampedValue<?> currentValue = manager.getValue(accessedValue.key);
			
			boolean isCurrentValueStale = currentValue.getAgeMilliseconds() > accessedValue.maxAge;
			
			if (isCurrentValueStale != accessedValue.wasStale) { return false; }
			
			if (!accessedValue.value.equals(currentValue.getValue())) { return false; }
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
		TimestampedValue<?> value = new TimestampedValue<>(executor.getValue(key));
		
		value.notifier = this;
		
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
	
	@Override
	public void notifyOfAccess(String key, Object value, long maxAge, boolean wasStale) {
		accessedValues.add(new AccessedValue(key, value, maxAge, wasStale));
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
