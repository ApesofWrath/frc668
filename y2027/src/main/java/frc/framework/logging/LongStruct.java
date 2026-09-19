package frc.framework.logging;

import java.nio.ByteBuffer;

public class LongStruct implements CustomStruct<Long> {
	public static LongStruct instance = new LongStruct();
	
	@Override
	public Long deserialize(ByteBuffer bb, LogReader logReader) {
		return bb.getLong();
	}
	
	@Override
	public Class<Long> getDataClass() {
		return Long.class;
	}
	
	@Override
	public String getTypeId() {
		return "long";
	}
	
	@Override
	public void serialize(Long value, LogWriter writer, ByteBuffer buffer) {
		buffer.putLong(value);
	}
}
