package frc.framework.logging;

import java.nio.ByteBuffer;

public class FloatStruct implements CustomStruct<Float> {
	public static FloatStruct instance = new FloatStruct();
	
	@Override
	public Float deserialize(ByteBuffer bb, LogReader logReader) {
		return bb.getFloat();
	}
	
	@Override
	public Class<Float> getDataClass() {
		return Float.class;
	}
	
	@Override
	public String getTypeId() {
		return "float";
	}
	
	@Override
	public void serialize(Float value, LogWriter writer, ByteBuffer buffer) {
		buffer.putFloat(value);
	}
}
