package frc.framework.logging;

import java.util.HashMap;

public class LogFrame {
	public HashMap<String, Object> data = new HashMap<>();
	
	public void set(String key, Object value) {
		data.put(key, value);
	}
}
