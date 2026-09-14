package frc.framework.systems;

public class ValueIdentifier<T> {
	private final String id;
	private final T defaultValue;
	
	public ValueIdentifier(String id, T defaultValue) {
		this.id = id;
		this.defaultValue = defaultValue;
	}
	
	public T getDefaultValue() {
		return defaultValue;
	}
	
	public String getId() {
		return id;
	}
}
