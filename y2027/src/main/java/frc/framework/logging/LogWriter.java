package frc.framework.logging;

import edu.wpi.first.util.datalog.DataLogWriter;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Paths;
import java.util.Calendar;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Objects;

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
		) + "-" + calendar.getTimeInMillis() + ".wpilog";
		String baseDir = RobotBase.isReal() ? "/home/lvuser/logs" : "logs";
		
		File dir = new File(baseDir);
		
		if (!dir.exists()) {
			if (!dir.mkdirs()) { throw new RuntimeException("Failed to create log directory"); }
		}
		
		return Paths.get(baseDir, fileName).toAbsolutePath().toString();
	}
	
	private final DataLogWriter dataLog;
	private final HashMap<LogField, Integer> existingFields = new HashMap<>();
	private final HashMap<Integer, Object> writtenValues = new HashMap<>();
	
	private long startTime = Long.MAX_VALUE;
	
	/**
	 * Create a log serializer
	 *
	 * @param stream The stream to write to
	 */
	public LogWriter(FileOutputStream stream) {
		dataLog = new DataLogWriter(stream);
	}
	
	/**
	 * Create a log serializer
	 *
	 * @param logPath The path to write to
	 */
	public LogWriter(String logPath) {
		try {
			dataLog = new DataLogWriter(logPath);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
	}
	
	
	/**
	 * Given a log frame, create the requisite delta entries and then add an entry to create a new log frame. Also known
	 * as, serialize a log frame.
	 *
	 * @param frame     The log frame to serialize.
	 * @param timestamp The timestamp to use for logging
	 */
	public synchronized void writeFrame(LogFrame frame, long timestamp) {
		int tickBoundary = dataLog.start("$TickBoundary", "int", "", timestamp);
		
		Packer.setupPackers();
		
		HashMap<String, Object> values = new HashMap<>();
		HashSet<LogField> entries = new HashSet<>();
		
		if (timestamp < startTime) {
			startTime = timestamp;
		}
		
		long time = timestamp - startTime;
		
		frame.data.forEach((key, value) -> {
			String typeId = DataLogUtils.getTypeId(DataLogUtils.normalizeValue(value));
			if (typeId != null) {
				values.put(key, value);
				entries.add(new LogField(key, typeId, "primitive"));
			} else {
				PackHelper helper = new PackHelper();
				
				helper.unpackFields(key, value, false);
				
				entries.addAll(helper.getFields());
				values.putAll(helper.getData());
			}
		});
		
		var removedEntries = new HashSet<>(existingFields.keySet());
		var newEntries = new HashSet<>(entries);
		
		newEntries.removeAll(existingFields.keySet());
		removedEntries.removeAll(entries);
		
		for (LogField removedEntry : removedEntries) {
			int id = existingFields.get(removedEntry);
			dataLog.finish(id, time);
			writtenValues.remove(id);
			existingFields.remove(removedEntry);
		}
		
		for (LogField addedEntry : newEntries) {
			int key = dataLog.start(addedEntry.id(), addedEntry.type(), addedEntry.metadata(), time);
			existingFields.put(addedEntry, key);
		}
		
		values.forEach((key, value) -> {
			LogField entry = null;
			
			for (LogField logField : existingFields.keySet()) {
				if (logField.id().equals(key)) {
					entry = logField;
					break;
				}
			}
			
			int id = existingFields.get(entry);
			
			if (!Objects.equals(writtenValues.get(id), value)) {
				writtenValues.put(id, value);
				
				DataLogUtils.appendValue(dataLog, id, DataLogUtils.normalizeValue(value), time);
			}
		});
		
		dataLog.finish(tickBoundary);
		
		dataLog.flush();
	}
}

