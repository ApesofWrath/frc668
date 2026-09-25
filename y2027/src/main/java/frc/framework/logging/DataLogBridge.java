package frc.framework.logging;

import edu.wpi.first.util.datalog.DataLogWriter;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Objects;

/**
 * A helper utility for writing data log files
 */
public class DataLogBridge {
	private final DataLogWriter dataLogWriter;
	private final ArrayList<String> startedFields = new ArrayList<>();
	private final HashMap<String, String> fieldTypes = new HashMap<>();
	private long minTime = 0;
	
	/**
	 * Create a utility for writing to a datalog
	 *
	 * @param dataLogWriter The dataLog to write to
	 */
	public DataLogBridge(DataLogWriter dataLogWriter) {
		this.dataLogWriter = dataLogWriter;
	}
	
	/**
	 * Write datalog data to disk
	 */
	public void flush() {
		dataLogWriter.flush();
	}
	
	private String getTypeId(Object value) {
		if (value instanceof Boolean) {
			return "boolean";
		}
		if (value instanceof Integer || value instanceof Long) {
			return "int64";
		}
		if (value instanceof Double) {
			return "double";
		}
		if (value instanceof String) {
			return "string";
		}
		
		if (value instanceof Boolean[]) {
			return "boolean[]";
		}
		if (value instanceof Integer[] || value instanceof Long[]) {
			return "int64[]";
		}
		if (value instanceof Double[]) {
			return "double[]";
		}
		if (value instanceof String[]) {
			return "string[]";
		}
		
		return null;
	}
	
	/**
	 * Write a log entry
	 *
	 * @param key       The key to set to
	 * @param value     The value to set it to
	 * @param timestamp The timestamp to store
	 */
	public void set(String key, Object value, long timestamp) {
		if (value instanceof ArrayList<?> arrayList) {
			set(key, arrayList.toArray(), timestamp);
			return;
		}
		
		int index = startedFields.indexOf(key);
		
		String typeId = getTypeId(value);
		
		if (typeId == null) { return; }
		
		if (index == -1) {
			fieldTypes.put(key, typeId);
			index = dataLogWriter.start(key, typeId);
			while (startedFields.size() <= index) {
				startedFields.add(null);
			}
			startedFields.set(index, key);
		}
		
		if (!Objects.equals(fieldTypes.get(key), typeId)) {
			System.err.println("Type of " + key + " is mismatched from initial type");
			return;
		}
		
		if (minTime == 0) {
			minTime = timestamp;
		}
		
		long time = timestamp - minTime;
		
		if (value instanceof Boolean bool) {
			dataLogWriter.appendBoolean(index, bool, time);
		}
		if (value instanceof Integer integer) {
			dataLogWriter.appendInteger(index, integer, time);
		}
		if (value instanceof Long int64) {
			dataLogWriter.appendInteger(index, int64, time);
		}
		if (value instanceof Double float64) {
			dataLogWriter.appendDouble(index, float64, time);
		}
		if (value instanceof String str) {
			dataLogWriter.appendString(index, str, time);
		}
		
		if (value instanceof boolean[] boolArr) {
			dataLogWriter.appendBooleanArray(index, boolArr, time);
		}
		if (value instanceof int[] integerArr) {
			dataLogWriter.appendIntegerArray(index, Arrays.stream(integerArr).asLongStream().toArray(), time);
		}
		if (value instanceof long[] int64Arr) {
			dataLogWriter.appendIntegerArray(index, int64Arr, time);
		}
		if (value instanceof double[] float64Arr) {
			dataLogWriter.appendDoubleArray(index, float64Arr, time);
		}
		if (value instanceof String[] strArr) {
			dataLogWriter.appendStringArray(index, strArr, time);
		}
	}
}
