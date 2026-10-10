package frc.framework.logging;

import edu.wpi.first.util.datalog.DataLogIterator;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;

import java.io.IOException;
import java.util.HashMap;

/**
 * Reads log files
 */
public class LogReader {
	private final DataLogReader reader;
	private final HashMap<String, Object> values = new HashMap<>();
	private final HashMap<Integer, LogField> fields = new HashMap<>();
	private final DataLogIterator iterator;
	
	/**
	 * Create a log reader using the DataLog reader instance
	 *
	 * @param reader The reader instance to use
	 */
	public LogReader(DataLogReader reader) {
		this.reader = reader;
		this.iterator = reader.iterator();
	}
	
	/**
	 * Create a log reader by opening a DataLog file on a given path
	 *
	 * @param path The path to the DataLog file to read
	 */
	public LogReader(String path) {
		try {
			reader = new DataLogReader(path);
		} catch (IOException e) {
			throw new RuntimeException(e);
		}
		this.iterator = reader.iterator();
	}
	
	/**
	 * Handle the effects of a given log record
	 *
	 * @param record The record to handle
	 *
	 * @return true if this is the end of a tick
	 */
	public boolean handleLogRecord(DataLogRecord record) {
		if (!record.isControl()) { // If we're setting a value, let's record that
			LogField field = fields.get(record.getEntry());
			Object value = DataLogUtils.getValue(field.type(), record);
			values.put(field.id(), value);
			return false; // Setting a value is not the end of a tick
		}
		
		if (record.isStart()) { // If we're creating a new record, lets store it
			DataLogRecord.StartRecordData startData = record.getStartData();
			
			fields.put(startData.entry, new LogField(startData.name, startData.type, startData.metadata));
			return false; // Starting a field is not the end of a tick
		}
		
		if (record.isFinish()) { // We're ending a field
			LogField field = fields.get(record.getFinishEntry());
			values.remove(field.id());
			fields.remove(record.getFinishEntry());
			return field.id().equals("$TickBoundary"); // If it's ending the tick boundary, it is the end of the tick
		}
		
		throw new RuntimeException("Unknown log record type " + record);
	}
	
	/**
	 * @return true if there is no records remaining in the log file
	 */
	public boolean isDone() {
		return !iterator.hasNext();
	}
	
	/**
	 * Try to read a log frame from the log file
	 *
	 * @return The log frame that was between the tick boundaries
	 */
	public LogFrame read() {
		while (true) {
			// If the log file is done, we are done
			if (!iterator.hasNext()) { return null; }
			
			// Get the next update in the log file
			DataLogRecord record = iterator.next();
			
			// If this is not a tick boundary, keep reading
			boolean isEndOfTick = handleLogRecord(record);
			
			if (!isEndOfTick) { continue; }
			
			// We've hit a tick boundary, let's decode it
			LogFrame result = new LogFrame();
			
			// Set up the pack helper
			PackHelper helper = new PackHelper();
			
			// Give the pack helper all the fields we know about
			fields.values().forEach(helper::addField);
			
			// Give the pack helper all the values we know about
			values.forEach(helper::setValue);
			
			// Let's decode root values
			fields.forEach((key, value) -> {
				// Primitives are root primitive values, packed::[packer_id] is a root packed value
				if (value.metadata().equals("primitive") || value.metadata().startsWith("packed::")) {
					result.set(value.id(), helper.getValueUntyped(value.id()));
				}
			});
			
			return result;
		}
	}
}
