package frc.framework.nodes;

/**
 * Classes implementing this interface can recieve notice when a value is read from a timestamped value
 */
public interface DataAccessNotifier {
	/**
	 * A value was read from a timestamped value
	 *
	 * @param value    The value stored
	 * @param maxAge   The maximum age of the value
	 * @param wasStale True if the value being read is stale at the time of reading
	 */
	void notifyOfAccess(Object value, long maxAge, boolean wasStale);
}
