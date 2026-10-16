package frc.framework.execution;

import frc.framework.systems.Priority;

/**
 * Represents a value with a given priority at a given time
 *
 * @param <T> The type of the value
 */
public class TimestampedValue<T> {
	private final long timestamp;
	private final Priority priority;
	private final T value;
	private final ExecutionManager manager;
	
	/**
	 * Create a TimestampedValue
	 *
	 * @param value     The value to store
	 * @param timestamp The timestamp that this value was set at
	 * @param priority  The priority that this value should take over other values
	 * @param manager   The current ExecutionManager managing this value
	 */
	public TimestampedValue(T value, long timestamp, Priority priority, ExecutionManager manager) {
		this.value = value;
		this.timestamp = timestamp;
		this.priority = priority;
		this.manager = manager;
	}
	
	/**
	 * @return The age of this value, in milliseconds
	 */
	public long getAgeMilliseconds() {
		return manager.getTimestamp() - timestamp;
	}
	
	/**
	 * @return The priority of this value
	 */
	public Priority getPriority() {
		return priority;
	}
	
	/**
	 * @return The timestamp that this value has
	 */
	public long getTimestamp() {
		return timestamp;
	}
	
	/**
	 * @return The value contained within this
	 */
	public Object getValue() {
		return value;
	}
	
	/**
	 * Returns the value if this value was updated this tick, otherwise, returns the default value.
	 *
	 * @param defaultValue The default value to return
	 *
	 * @return the default value if this was set in a previous tick, otherwise the value
	 */
	public T orDefault(T defaultValue) {
		return orDefault(defaultValue, 0);
	}
	
	/**
	 * Returns the value in this container, unless it's older than maxAge milliseconds, in which case, returns the
	 * default value.
	 *
	 * @param defaultValue The default value.
	 * @param maxAge       The maximum age of the value.
	 *
	 * @return defaultValue if value is older than maxAge milliseconds, otherwise returns the value.
	 */
	public T orDefault(T defaultValue, long maxAge) {
		if (getAgeMilliseconds() > maxAge) {
			return defaultValue;
		}
		return value;
	}
	
	/**
	 * Returns the value if this value was updated this tick, otherwise, returns null
	 *
	 * @return the value if it was set this tick, otherwise, null
	 */
	public T orNull() {
		return orNull(0);
	}
	
	/**
	 * Returns the value in this container, unless it's older than maxAge milliseconds, in which case, returns null
	 *
	 * @param maxAge The maximum age of the value.
	 *
	 * @return null if value is older than maxAge milliseconds, otherwise returns the value.
	 */
	public T orNull(long maxAge) {
		if (getAgeMilliseconds() > maxAge) {
			return null;
		}
		return value;
	}
	
	/**
	 * Determine if this value should override another TimestampedValue
	 * <p>
	 * Newer always beats older, higher priority then beats lower priority, if priority and timestamp are identical,
	 * returns false.
	 *
	 * @param other The other TimestampedValue
	 *
	 * @return True if this value should take precedence over the other value
	 */
	public boolean takesPrecedenceOver(TimestampedValue<?> other) {
		if (this.timestamp > other.timestamp) {
			return true;
		}
		if (other.timestamp > this.timestamp) {
			return false;
		}
		
		return this.priority.value > other.priority.value;
	}
}
