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

/**
 * Handles serialization of log data
 */
public class LogWriter {
	/**
	 * Get the primary logging directory
	 *
	 * @return The logging directory to use
	 */
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
	
	/**
	 * Create a log writer and write the header
	 *
	 * @param path The path to write the log to
	 *
	 * @return The log writer
	 */
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
	
	/**
	 * Create a log serializer
	 *
	 * @param stream The stream to write output data to
	 */
	public LogWriter(FileOutputStream stream) {
		outputStream = new DataOutputStream(stream);
	}
	
	/**
	 * Convert a raw value into a protobuf message representing the value
	 *
	 * @param obj The value to encode
	 *
	 * @return The protobuf message
	 */
	public Value encodeValue(Object obj) {
		for (CustomStruct<?, ?> struct : Logging.customStructs) {
			Value serialized = tryEncodeStruct(obj, struct);
			
			if (serialized != null) { return serialized; }
		}
		
		throw new RuntimeException("Cannot serialize value " + obj);
	}
	
	/**
	 * Get a string index in the string table, writing an entry to add it to the table if it is not part of the table.
	 *
	 * @param str The string to search for and add to in the string table
	 *
	 * @return The index of the string
	 */
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
	
	/**
	 * Given a log frame, create the requisite delta entries and then add an entry to create a new log frame. Also known
	 * as, serialize a log frame.
	 *
	 * @param frame The log frame to serialize.
	 */
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
