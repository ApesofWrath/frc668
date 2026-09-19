package frc.framework.logging;

import java.nio.ByteBuffer;

public class IntegerStruct implements CustomStruct<Integer> {
	public static IntegerStruct instance = new IntegerStruct();
	
	@Override
	public Integer deserialize(ByteBuffer bb, LogReader logReader) {
		return bb.getInt();
	}
	
	@Override
	public Class<Integer> getDataClass() {
		return Integer.class;
	}
	
	@Override
	public String getTypeId() {
		return "int";
	}
	
	@Override
	public void serialize(Integer value, LogWriter writer, ByteBuffer buffer) {
		buffer.putInt(value);
	}
}
