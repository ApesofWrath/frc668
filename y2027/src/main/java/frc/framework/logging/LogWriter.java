package frc.framework.logging;

import com.google.protobuf.ByteString;
import com.google.protobuf.CodedOutputStream;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Objects;
import java.util.Set;

import static frc.framework.logging.Logging.customStructs;
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
	
	private byte[] encodeValue(Object obj) {
		ByteBuffer bb = ByteBuffer.allocate(1024);
		writeValueToBB(obj, bb);
		byte[] result = new byte[bb.position()];
		bb.position(0);
		bb.get(result);
		return result;
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
	
	private <T> boolean tryWriteStruct(Object obj, ByteBuffer buffer, CustomStruct<T> struct) {
		if (struct.getDataClass().isInstance(obj)) {
			String id = struct.getTypeId();
			int typeIdIdx = getStringId(id);
			
			buffer.putInt(typeIdIdx);
			
			//noinspection unchecked
			struct.serialize((T) obj, this, buffer);
			
			return true;
		}
		
		return false;
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
	
	public void writeFrame(LogFrame frame) {
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
							.setSetKey(SetKeyLogEntry.newBuilder().setBuffer(ByteString.copyFrom(encodeValue(value))))
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
	
	public void writeValueToBB(Object obj, ByteBuffer buffer) {
		for (CustomStruct<?> struct : customStructs) {
			if (tryWriteStruct(obj, buffer, struct)) { return; }
		}
		
		throw new RuntimeException("Aw shucks! We couldn't serialize the value " + obj);
	}
}
