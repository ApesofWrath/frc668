package frc.framework.systems;

import java.util.HashMap;

import static java.text.MessageFormat.format;

public class ValueIdentifier<T> {
	private static final HashMap<String, ValueIdentifier<?>> singletons = new HashMap<>();
	
	@SuppressWarnings(
		"unchecked"
	)
	public static <T> ValueIdentifier<T> get(String id) {
		if (singletons.containsKey(id)) {
			return (ValueIdentifier<T>) singletons.get(id);
		}
		throw new RuntimeException(format("Cannot find value identifier with id {0}", id));
	}
	
	@SuppressWarnings(
		"unchecked"
	)
	public static <T> ValueIdentifier<T> get(String id, T defaultValue) {
		if (singletons.containsKey(id)) {
			return (ValueIdentifier<T>) singletons.get(id);
		}
		ValueIdentifier<T> instance = new ValueIdentifier<>(id, defaultValue);
		singletons.put(id, instance);
		return instance;
	}
	
	private final String id;
	
	private final T defaultValue;
	
	private ValueIdentifier(String id, T defaultValue) {
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
