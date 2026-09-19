package frc.framework.logging;

import java.nio.ByteBuffer;

public class ArrayStruct implements CustomStruct<Object[]> {
	public static ArrayStruct instance = new ArrayStruct();
	
	@Override
	public Object[] deserialize(ByteBuffer bb, LogReader logReader) {
		int length = bb.getInt();
		
		Object[] result = new Object[length];
		
		for (int i = 0; i < length; i++) {
			result[i] = logReader.decodeValue(bb);
		}
		
		return result;
	}
	
	@Override
	public Class<Object[]> getDataClass() {
		//noinspection unchecked
		return (Class<Object[]>) Object.class.arrayType();
	}
	
	@Override
	public String getTypeId() {
		return "array";
	}
	
	@Override
	public void serialize(Object[] value, LogWriter writer, ByteBuffer buffer) {
		buffer.putInt(value.length);
		
		for (Object o : value) {
			writer.writeValueToBB(o, buffer);
		}
	}
}
