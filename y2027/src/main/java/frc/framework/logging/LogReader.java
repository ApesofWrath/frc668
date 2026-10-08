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
	
	public LogReader(DataLogReader reader) {
		this.reader = reader;
		this.iterator = reader.iterator();
	}
	
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
		if (!record.isControl()) {
			LogField field = fields.get(record.getEntry());
			Object value = DataLogUtils.getValue(field.type(), record);
			values.put(field.id(), value);
			return false;
		}
		
		if (record.isStart()) {
			DataLogRecord.StartRecordData startData = record.getStartData();
			
			fields.put(startData.entry, new LogField(startData.name, startData.type, startData.metadata));
			return false;
		}
		
		if (record.isFinish()) {
			LogField field = fields.get(record.getFinishEntry());
			values.remove(field.id());
			fields.remove(record.getFinishEntry());
			return field.id().equals("$TickBoundary");
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
	 */
	public LogFrame read() {
		while (true) {
			if (!iterator.hasNext()) { return null; }
			
			DataLogRecord record = iterator.next();
			
			boolean isEndOfTick = handleLogRecord(record);
			
			if (!isEndOfTick) { continue; }
			
			LogFrame result = new LogFrame();
			
			PackHelper helper = new PackHelper();
			
			fields.forEach((key, value) -> {
				helper.addField(value);
			});
			
			values.forEach(helper::setValue);
			
			fields.forEach((key, value) -> {
				if (value.metadata().equals("primitive") || value.metadata().startsWith("packed::")) {
					result.set(value.id(), helper.getValueUntyped(value.id()));
				}
			});
			
			return result;
		}
	}
}
