package frc.framework.logging;

/**
 * Handles protobuf serialization &amp; deserialization for arrays
 */
public class ArrayStruct implements CustomStruct<Object[], SystemLog.Array> {
	/**
	 * Singleton
	 */
	public static ArrayStruct instance = new ArrayStruct();
	
	@Override
	public Object[] deserialize(SystemLog.Array array, LogReader reader) {
		Object[] result = new Object[array.getItemsCount()];
		for (int i = 0; i < result.length; i++) {
			result[i] = reader.decodeValue(array.getItems(i));
		}
		return result;
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.ARRAY_FIELD_NUMBER;
	}
	
	@Override
	public Class<Object[]> getUnserializedClass() {
		//noinspection unchecked
		return (Class<Object[]>) Object.class.arrayType();
	}
	
	@Override
	public SystemLog.Value serialize(Object[] value, LogWriter writer) {
		SystemLog.Array.Builder builder = SystemLog.Array.newBuilder();
		
		for (Object item : value) {
			builder.addItems(writer.encodeValue(item));
		}
		
		return SystemLog.Value.newBuilder().setArray(builder).build();
	}
}
