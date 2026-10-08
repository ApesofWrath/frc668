package frc.framework.logging;

/**
 * An entry type in a WPILog file
 *
 * @param id
 * @param type
 * @param metadata
 */
public record LogField(String id, String type, String metadata) {
	/**
	 * Create a log entry with no metadata
	 *
	 * @param id   The log entry ID
	 * @param type The type fo the log entry
	 */
	public LogField(String id, String type) {
		this(id, type, "");
	}
}
