package frc.framework.logging;

public class FloatStruct implements CustomStruct<Float, Float> {
	public static FloatStruct instance = new FloatStruct();
	
	@Override
	public Float deserialize(Float value, LogReader reader) {
		return value;
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.FLOAT_FIELD_NUMBER;
	}
	
	@Override
	public Class<Float> getUnserializedClass() {
		return Float.class;
	}
	
	@Override
	public SystemLog.Value serialize(Float value, LogWriter writer) {
		return SystemLog.Value.newBuilder().setFloat(value).build();
	}
}
