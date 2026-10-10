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
	 * @param timestamp The timestamp to use for logging
	 * @param frame     The log frame to serialize.
	 */
	public synchronized void writeFrame(long timestamp, LogFrame frame) {
		// WPILog files want 0 to be the start of the log file, so let's make startTime our "zero time"
		// and have "time", which is normalized so that we start at zero, as timestamp is currently the UNIX timestamp
		if (timestamp < startTime) {
			startTime = timestamp;
		}
		long time = (timestamp - startTime) * 1000;
		
		// This one's a doozy, so let's get started
		// Indicate that we are starting a tick
		int tickBoundary = dataLog.start("$TickBoundary", "int", "", time);
		
		// Make sure that all packers are initialized
		Packer.setupPackers();
		
		// This is our desired set of values
		HashMap<String, Object> values = new HashMap<>();
		// This is our desired set of metadata (e.g, name and type and metadata)
		HashSet<LogField> entries = new HashSet<>();
		
		// Let's try to unpack all of our data
		frame.data.forEach((key, value) -> {
			String typeId = DataLogUtils.getTypeId(DataLogUtils.normalizeValue(value));
			
			// If this is a primitive type, save it, and mark it as a primitive root
			if (typeId != null) {
				values.put(key, value);
				entries.add(new LogField(key, typeId, "primitive"));
			} else {
				// Otherwise, let's unpack it as a packed root
				PackHelper helper = new PackHelper();
				
				helper.unpackFields(key, value, false);
				
				entries.addAll(helper.getFields());
				values.putAll(helper.getData());
			}
		});
		
		// Figure out the entries we need to add and remove
		var removedEntries = new HashSet<>(existingFields.keySet());
		var newEntries = new HashSet<>(entries);
		
		newEntries.removeAll(existingFields.keySet());
		removedEntries.removeAll(entries);
		
		// Sync entries with what we want
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
		
		// Push new values
		values.forEach((key, value) -> {
			// Figure out the integer ID of this entry
			LogField entry = null;
			
			for (LogField logField : existingFields.keySet()) {
				if (logField.id().equals(key)) {
					entry = logField;
					break;
				}
			}
			
			int id = existingFields.get(entry);
			
			// If the value has changed, write it
			if (!Objects.equals(writtenValues.get(id), value)) {
				writtenValues.put(id, value);
				
				DataLogUtils.appendValue(dataLog, id, DataLogUtils.normalizeValue(value), time);
			}
		});
		
		// Say that this tick is over
		dataLog.finish(tickBoundary, time);
		
		// Write it to disk
		dataLog.flush();
	}
}

