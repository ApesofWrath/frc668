package frc.framework.logging;

public interface CustomStruct<TData, TMessage> {
	TData deserialize(TMessage message, LogReader reader);
	
	int getFieldIndex();
	
	Class<TData> getUnserializedClass();
	
	SystemLog.Value serialize(TData value, LogWriter writer);
}
