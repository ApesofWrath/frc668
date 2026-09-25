package frc.framework.logging;

/**
 * A type that can be serialized and deserialized from protobuf
 *
 * @param <TData>    The deserialized value type
 * @param <TMessage> The protobuf message type
 */
public interface CustomStruct<TData, TMessage> {
	/**
	 * Convert a protobuf message into data
	 *
	 * @param message The protobuf message
	 * @param reader  The log reader instance, used for strings
	 *
	 * @return The decoded value
	 */
	TData deserialize(TMessage message, LogReader reader);
	
	/**
	 * The protobuf field index of the field on the Value message
	 *
	 * @return The field index
	 */
	int getFieldIndex();
	
	/**
	 * Get the type of the deserialized value, used for determining which CustomStruct to route to
	 *
	 * @return The reflection class
	 */
	Class<TData> getUnserializedClass();
	
	/**
	 * Convert raw data to a Value message to be stored to a log file
	 *
	 * @param value  The value to encode into a message
	 * @param writer The log writer instance, used for strings
	 *
	 * @return The protobuf message
	 */
	SystemLog.Value serialize(TData value, LogWriter writer);
}
