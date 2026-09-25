package frc.framework.logging;

/**
 * Handles protobuf serialization &amp; deserialization for doubles
 */
public class DoubleStruct implements CustomStruct<Double, Double> {
	/**
	 * Singleton
	 */
	public static DoubleStruct instance = new DoubleStruct();
	
	@Override
	public Double deserialize(Double value, LogReader reader) {
		return value;
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.DOUBLE_FIELD_NUMBER;
	}
	
	@Override
	public Class<Double> getUnserializedClass() {
		return Double.class;
	}
	
	@Override
	public SystemLog.Value serialize(Double value, LogWriter writer) {
		return SystemLog.Value.newBuilder().setDouble(value).build();
	}
}
