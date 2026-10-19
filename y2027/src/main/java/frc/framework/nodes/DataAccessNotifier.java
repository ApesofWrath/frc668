package frc.framework.nodes;

/**
 * Classes implementing this interface can recieve notice when a value is read from a timestamped value
 */
public interface DataAccessNotifier {
	/**
	 * A value was read from a timestamped value
	 *
	 * @param key      The key
	 * @param value    The value stored
	 * @param maxAge   The maximum age of the value
	 * @param wasStale
	 */
	void notifyOfAccess(String key, Object value, long maxAge, boolean wasStale);
}
