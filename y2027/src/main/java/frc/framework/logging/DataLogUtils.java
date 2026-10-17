package frc.framework.logging;

import edu.wpi.first.util.datalog.DataLogRecord;
import edu.wpi.first.util.datalog.DataLogWriter;

import java.util.ArrayList;
import java.util.Arrays;

public class DataLogUtils {
	/**
	 * Append a value to a datalog
	 *
	 * @param writer    The writer to append the value to
	 * @param id        The id of the field to append to
	 * @param value     The value to append
	 * @param timestamp The timestamp to store to the log file
	 */
	public static void appendValue(DataLogWriter writer, int id, Object value, long timestamp) {
		if (value instanceof String v) {
			writer.appendString(id, v, timestamp);
		} else if (value instanceof Boolean v) {
			writer.appendBoolean(id, v, timestamp);
		} else if (value instanceof Long v) {
			writer.appendInteger(id, v, timestamp);
		} else if (value instanceof Double v) {
			writer.appendDouble(id, v, timestamp);
		} else if (value instanceof Float v) {
			writer.appendFloat(id, v, timestamp);
		} else if (value instanceof String[] v) {
			writer.appendStringArray(id, v, timestamp);
		} else if (value instanceof boolean[] v) {
			writer.appendBooleanArray(id, v, timestamp);
		} else if (value instanceof long[] v) {
			writer.appendIntegerArray(id, v, timestamp);
		} else if (value instanceof double[] v) {
			writer.appendDoubleArray(id, v, timestamp);
		} else if (value instanceof float[] v) {
			writer.appendFloatArray(id, v, timestamp);
		} else {
			throw new RuntimeException("Non-primitive value " + value);
		}
	}
	
	/**
	 * Get the datalog type id for a given value
	 *
	 * @param value The value to get the type ID of
	 *
	 * @return The type ID
	 */
	public static String getTypeId(Object value) {
		if (value instanceof String) {
			return "string";
		} else if (value instanceof Boolean) {
			return "boolean";
		} else if (value instanceof Long) {
			return "int64";
		} else if (value instanceof Double) {
			return "double";
		} else if (value instanceof Float) {
			return "float";
		} else if (value instanceof String[]) {
			return "string[]";
		} else if (value instanceof Boolean[]) {
			return "boolean[]";
		} else if (value instanceof Long[]) {
			return "int64[]";
		} else if (value instanceof Double[]) {
			return "double[]";
		} else if (value instanceof Float[]) {
			return "float[]";
		}
		
		return null;
	}
	
	/**
	 * Get the value from a datalog entry
	 *
	 * @param type  The type to read
	 * @param entry The entry to read from
	 *
	 * @return The read value
	 */
	public static Object getValue(String type, DataLogRecord entry) {
		return switch (type) {
		case "string" -> entry.getString();
		case "string[]" -> entry.getStringArray();
		case "int64" -> entry.getInteger();
		case "int64[]" -> entry.getIntegerArray();
		case "float" -> entry.getFloat();
		case "float[]" -> entry.getFloatArray();
		case "double" -> entry.getDouble();
		case "double[]" -> entry.getDoubleArray();
		case "boolean" -> entry.getBoolean();
		case "boolean[]" -> entry.getBooleanArray();
		default -> throw new RuntimeException("Unknown value type " + type);
		};
	}
	
	/**
	 * Join several key paths together
	 *
	 * @param segments The key paths to join
	 *
	 * @return The resulting key path
	 */
	public static String joinKey(String... segments) {
		ArrayList<String> result = new ArrayList<>();
		
		for (String segment : segments) {
			for (String subsegment : segment.split("/")) {
				if (subsegment.isEmpty()) { continue; }
				
				result.add(subsegment);
			}
		}
		
		return "/" + String.join("/", result);
	}
	
	/**
	 * Convert primitive types in Java to primitive types in datalog
	 *
	 * @param value The value to upcast
	 *
	 * @return The upcast value
	 */
	public static Object normalizeValue(Object value) {
		if (value instanceof Integer integer) {
			return (long) integer;
		} else if (value instanceof int[] arr) {
			return Arrays.stream(arr).mapToLong(item -> (long) item).toArray();
		}
		return value;
	}
	
	private DataLogUtils() {
	}
}
