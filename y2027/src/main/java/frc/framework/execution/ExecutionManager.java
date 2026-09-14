package frc.framework.execution;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.framework.systems.ValueIdentifier;

import java.util.HashMap;

public class ExecutionManager {
	private final HashMap<String, Object> values = new HashMap<>();
	private final HashMap<String, Integer> valuePriorities = new HashMap<>();
	private final HashMap<System, SystemCachableResult> cachedResults = new HashMap<>();
	public ExecutionPlan plan;
	
	public void execute(long time) {
		// reset values
		values.clear();
		valuePriorities.clear();
		
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
	
	@SuppressWarnings(
		"unchecked"
	)
	public <T> T getValue(String id) {
		if (!values.containsKey(id)) {
			values.put(id, ValueIdentifier.get(id).getDefaultValue());
		}
		
		return (T) values.get(id);
	}
	
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
	
	public void trySetValue(String key, Object value, int priority) {
		int currentPriority = valuePriorities.getOrDefault(key, Integer.MIN_VALUE);
		
		if (priority > currentPriority) {
			values.put(key, value);
			valuePriorities.put(key, priority);
		}
	}
}
