package frc.framework.logging;

public class IntegerStruct implements CustomStruct<Integer, Integer> {
	public static IntegerStruct instance = new IntegerStruct();
	
	@Override
	public Integer deserialize(Integer value, LogReader reader) {
		return value;
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.INTEGER_FIELD_NUMBER;
	}
	
	@Override
	public Class<Integer> getUnserializedClass() {
		return Integer.class;
	}
	
	@Override
	public SystemLog.Value serialize(Integer value, LogWriter writer) {
		return SystemLog.Value.newBuilder().setInteger(value).build();
	}
}
