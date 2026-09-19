package frc.framework.logging;

import edu.wpi.first.util.struct.Struct;

import java.nio.ByteBuffer;

public class StructUtils {
	public static <T> CustomStruct<T> toCustomStruct(Struct<T> struct) {
		return new CustomStruct<T>() {
			
			@Override
			public T deserialize(ByteBuffer bb, LogReader logReader) {
				return struct.unpack(bb);
			}
			
			@Override
			public Class<T> getDataClass() {
				return struct.getTypeClass();
			}
			
			@Override
			public String getTypeId() {
				return struct.getTypeName();
			}
			
			@Override
			public void serialize(T value, LogWriter writer, ByteBuffer buffer) {
				struct.pack(buffer, value);
			}
		};
	}
}
