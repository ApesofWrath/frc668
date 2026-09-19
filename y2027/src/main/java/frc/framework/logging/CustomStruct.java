package frc.framework.logging;

import java.nio.ByteBuffer;

public interface CustomStruct<T> {
	T deserialize(ByteBuffer bb, LogReader logReader);
	
	Class<T> getDataClass();
	
	String getTypeId();
	
	void serialize(T value, LogWriter writer, ByteBuffer buffer);
}
