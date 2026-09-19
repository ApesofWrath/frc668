package frc.framework.logging;

import java.nio.ByteBuffer;

public class DoubleStruct implements CustomStruct<Double> {
	public static DoubleStruct instance = new DoubleStruct();
	
	@Override
	public Double deserialize(ByteBuffer bb, LogReader logReader) {
		return bb.getDouble();
	}
	
	@Override
	public Class<Double> getDataClass() {
		return Double.class;
	}
	
	@Override
	public String getTypeId() {
		return "double";
	}
	
	@Override
	public void serialize(Double value, LogWriter writer, ByteBuffer buffer) {
		buffer.putDouble(value);
	}
}
