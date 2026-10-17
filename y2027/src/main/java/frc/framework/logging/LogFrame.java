package frc.framework.logging;

import java.util.HashMap;

/**
 * The state of logged keys and values at a given tick
 */
public class LogFrame {
	/**
	 * The key-value set
	 */
	public HashMap<String, Object> data = new HashMap<>();
	
	/**
	 * Store a value for this logging frame
	 *
	 * @param key   The key to store the value into
	 * @param value THe value to store
	 */
	public void set(String key, Object value) {
		data.put(key, value);
	}
}
