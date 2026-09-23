package frc.framework.logging;

import com.google.protobuf.CodedOutputStream;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Objects;
import java.util.Set;

import static frc.framework.logging.SystemLog.*;

public class LogWriter {
	public static String getLogPath() {
		Calendar calendar = Calendar.getInstance();
		String fileName = calendar.get(Calendar.YEAR) + "-" + (calendar.get(Calendar.MONTH) + 1) + "-" + calendar.get(
			Calendar.DAY_OF_MONTH
		) + "-" + calendar.getTimeInMillis() + ".slog";
		String baseDir = RobotBase.isReal() ? "/home/lvuser/logs" : "logs";
		
		File dir = new File(baseDir);
		
		if (!dir.exists()) {
			if (!dir.mkdirs()) { throw new RuntimeException("Failed to create log directory"); }
		}
		
		return Paths.get(baseDir, fileName).toAbsolutePath().toString();
	}
	
	public static LogWriter open(String path) {
		try {
			LogWriter writer = new LogWriter(new FileOutputStream(path));
			
			writer.writeHeader();
			
			return writer;
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	private final DataOutputStream outputStream;
	
	private final ArrayList<String> stringTable = new ArrayList<>();
	
	private LogFrame previouslyWrittenLogFrame = new LogFrame();
	
	public LogWriter(FileOutputStream stream) {
		outputStream = new DataOutputStream(stream);
	}
	
	public Value encodeValue(Object obj) {
		for (CustomStruct<?, ?> struct : Logging.customStructs) {
			Value serialized = tryEncodeStruct(obj, struct);
			
			if (serialized != null) { return serialized; }
		}
		
		throw new RuntimeException("Cannot serialize value " + obj);
	}
	
	public int getStringId(String str) {
		int index = stringTable.indexOf(str);
		
		if (index == -1) {
			index = stringTable.size();
			write(
				LogEntry.newBuilder()
					.setDefineString(DefineStringLogEntry.newBuilder().setIndex(index).setMessage(str))
					.build()
			);
			stringTable.add(str);
		}
		
		return index;
	}
	
	private <T, TOut> Value tryEncodeStruct(Object obj, CustomStruct<T, TOut> struct) {
		if (struct.getUnserializedClass().isInstance(obj)) {
			//noinspection unchecked
			return struct.serialize((T) obj, this);
		}
		
		return null;
	}
	
	private void write(LogEntry entry) {
		try {
			byte[] data = new byte[1024];
			CodedOutputStream outp = CodedOutputStream.newInstance(data);
			entry.writeTo(outp);
			
			outputStream.writeInt(outp.getTotalBytesWritten());
			outputStream.write(data, 0, outp.getTotalBytesWritten());
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	public synchronized void writeFrame(LogFrame frame) {
		Set<String> removedKeys = previouslyWrittenLogFrame.data.keySet();
		
		removedKeys.removeAll(frame.data.keySet());
		
		for (String removedKey : removedKeys) {
			stringTable.remove(removedKey);
			write(
				LogEntry.newBuilder().setDeleteKey(DeleteKeyEntry.newBuilder().setKey(getStringId(removedKey))).build()
			);
		}
		
		for (String key : frame.data.keySet()) {
			Object value = frame.data.get(key);
			Object previousValue = previouslyWrittenLogFrame.data.get(key);
			
			if (!Objects.deepEquals(value, previousValue)) {
				try {
					write(
						LogEntry.newBuilder()
							.setSetKey(
								SetKeyLogEntry.newBuilder().setKeyIndex(getStringId(key)).setValue(encodeValue(value))
							)
							.build()
					);
				} catch (Exception e) {
					System.out.println("failed to write key " + key);
					//noinspection CallToPrintStackTrace
					e.printStackTrace();
				}
			}
		}
		
		write(LogEntry.newBuilder().setFinishFrame(FinishFrameEntry.newBuilder()).build());
		
		previouslyWrittenLogFrame = frame;
	}
	
	private void writeHeader() {
		writeUtf8(
			"This is a SystemLog file, it is not designed to be read by humans!\nIt's designed to be read by computers.\nUse something like InfoDeck to read this file!"
		);
	}
	
	@SuppressWarnings(
		"SameParameterValue"
	)
	private void writeUtf8(String str) {
		byte[] bytes = str.getBytes(StandardCharsets.UTF_8);
		
		try {
			outputStream.writeInt(bytes.length);
			outputStream.write(bytes);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
}
