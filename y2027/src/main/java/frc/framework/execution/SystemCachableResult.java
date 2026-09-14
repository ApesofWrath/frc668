package frc.framework.execution;

import java.util.HashMap;
import java.util.Map.Entry;

public class SystemCachableResult {
	public long timeCreated;
	public HashMap<String, Object> values = new HashMap<>();
	public HashMap<String, Integer> priorities = new HashMap<>();
	public HashMap<String, Object> accessedValues = new HashMap<>();
	
	public SystemCachableResult(long time) {
		this.timeCreated = time;
	}
	
	public void apply(ExecutionManager manager) {
		for (String key : values.keySet()) {
			Object value = values.get(key);
			int priority = priorities.get(key);
			
			manager.trySetValue(key, value, priority);
		}
	}
	
	public boolean doInputsMatchWithManager(ExecutionManager manager) {
		for (Entry<String, Object> entry : accessedValues.entrySet()) {
			Object currentValue = manager.getValue(entry.getKey());
			
			if (!currentValue.equals(entry.getValue())) {
				return false;
			}
		}
		
		return true;
	}
	
	public Object get(ExecutionManager executor, String key) {
		Object value = executor.getValue(key);
		accessedValues.put(key, value);
		return value;
	}
	
	public boolean isFresh(int time, int millisecondThreshold) {
		return (time - timeCreated) < millisecondThreshold;
	}
	
	public void set(String key, Object value, int priority) {
		values.put(key, value);
		priorities.put(key, priority);
	}
}
