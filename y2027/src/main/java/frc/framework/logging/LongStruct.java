package frc.framework.logging;

/**
 * Handles protobuf serialization &amp; deserialization for longs
 */
public class LongStruct implements CustomStruct<Long, Long> {
	/**
	 * Singleton
	 */
	public static LongStruct instance = new LongStruct();
	
	@Override
	public Long deserialize(Long value, LogReader reader) {
		return value;
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.LONG_FIELD_NUMBER;
	}
	
	@Override
	public Class<Long> getUnserializedClass() {
		return Long.class;
	}
	
	@Override
	public SystemLog.Value serialize(Long value, LogWriter writer) {
		return SystemLog.Value.newBuilder().setLong(value).build();
	}
}
