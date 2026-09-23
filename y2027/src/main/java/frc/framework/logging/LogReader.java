package frc.framework.logging;

import com.google.protobuf.Descriptors;

import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.ArrayList;
import java.util.HashMap;

public class LogReader {
	public static LogReader open(String path) {
		try {
			LogReader reader = new LogReader(new FileInputStream(path));
			
			reader.readHeader();
			
			return reader;
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	public final ArrayList<String> stringTable = new ArrayList<>();
	private final ByteBuffer primaryBuffer;
	private final HashMap<String, Object> data = new HashMap<>();
	
	public ArrayList<LogFrame> frames = new ArrayList<>();
	
	public LogReader(FileInputStream stream) {
		try {
			primaryBuffer = ByteBuffer.wrap(stream.readAllBytes());
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	public void decode() {
		while (primaryBuffer.hasRemaining()) {
			SystemLog.LogEntry entry = readEntry();
			
			if (entry.hasDefineString()) {
				SystemLog.DefineStringLogEntry defString = entry.getDefineString();
				while (stringTable.size() <= defString.getIndex()) {
					stringTable.add(null);
				}
				stringTable.set(defString.getIndex(), defString.getMessage());
			}
			
			if (entry.hasDeleteKey()) {
				data.remove(stringTable.get(entry.getDeleteKey().getKey()));
			}
			
			if (entry.hasFinishFrame()) {
				LogFrame frame = new LogFrame();
				frame.data.putAll(data);
				frames.add(frame);
			}
			
			if (entry.hasSetKey()) {
				SystemLog.SetKeyLogEntry setKey = entry.getSetKey();
				
				data.put(stringTable.get(setKey.getKeyIndex()), decodeValue(setKey.getValue()));
			}
		}
	}
	
	public Object decodeValue(SystemLog.Value value) {
		for (CustomStruct<?, ?> customStruct : Logging.customStructs) {
			Object decoded = tryDeserializeStruct(value, customStruct);
			
			if (decoded != null) { return decoded; }
		}
		
		throw new RuntimeException("Failed to decode value " + value);
	}
	
	private SystemLog.LogEntry readEntry() {
		try {
			int length = primaryBuffer.getInt();
			byte[] bytes = new byte[length];
			primaryBuffer.get(bytes);
			
			return SystemLog.LogEntry.parseFrom(bytes);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	private void readHeader() {
		String header = readUtfString();
		
		if (!header.contains("SystemLog")) {
			throw new RuntimeException("INVALID LOG FILE");
		}
	}
	
	public String readUtfString() {
		int length = primaryBuffer.getInt();
		byte[] bytes = new byte[length];
		primaryBuffer.get(bytes);
		
		return new String(bytes, StandardCharsets.UTF_8);
	}
	
	private <TMessage, TValue> TValue tryDeserializeStruct(
		SystemLog.Value value, CustomStruct<TValue, TMessage> struct
	) {
		Descriptors.FieldDescriptor descriptor = SystemLog.Value.getDescriptor().getField(struct.getFieldIndex() - 1); // Java is zero-indexed, whilst protobuf is one-indexed
		if (value.hasField(descriptor)) {
			//noinspection unchecked
			return struct.deserialize((TMessage) value.getField(descriptor), this);
		}
		return null;
	}
}
