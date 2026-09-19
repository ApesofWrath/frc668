package frc.framework.logging;

import java.nio.ByteBuffer;

public class StringStruct implements CustomStruct<String> {
	public static StringStruct instance = new StringStruct();
	
	@Override
	public String deserialize(ByteBuffer bb, LogReader logReader) {
		return logReader.stringTable.get(bb.getInt());
	}
	
	@Override
	public Class<String> getDataClass() {
		return String.class;
	}
	
	@Override
	public String getTypeId() {
		return "string";
	}
	
	@Override
	public void serialize(String value, LogWriter writer, ByteBuffer buffer) {
		buffer.putInt(writer.getStringId(value));
	}
}
