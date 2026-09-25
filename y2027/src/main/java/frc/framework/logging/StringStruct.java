package frc.framework.logging;

/**
 * Handles protobuf serialization &amp; deserialization for strings
 */
public class StringStruct implements CustomStruct<String, SystemLog.StringReference> {
	/**
	 * Singleton
	 */
	public static StringStruct instance = new StringStruct();
	
	@Override
	public String deserialize(SystemLog.StringReference stringValue, LogReader reader) {
		return reader.stringTable.get(stringValue.getIndex());
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.STR_FIELD_NUMBER;
	}
	
	@Override
	public Class<String> getUnserializedClass() {
		return String.class;
	}
	
	@Override
	public SystemLog.Value serialize(String value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setStr(SystemLog.StringReference.newBuilder().setIndex(writer.getStringId(value)))
			.build();
	}
}
