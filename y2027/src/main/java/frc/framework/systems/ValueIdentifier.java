package frc.framework.systems;

public class ValueIdentifier<T> {
	private String id;
	private T defaultValue;

	public T getDefaultValue() {
		return defaultValue;
	}

	public String getId() {
		return id;
	}

	public ValueIdentifier(String id, T defaultValue) {
		this.id = id;
		this.defaultValue = defaultValue;
	}
}
