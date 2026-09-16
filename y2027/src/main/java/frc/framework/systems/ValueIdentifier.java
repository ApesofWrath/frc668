package frc.framework.systems;

import java.util.HashMap;

import static java.text.MessageFormat.format;

/**
 * Identifies slots to contain values in the value graph
 *
 * @param <T> The type of the data contained within the slots
 */
public class ValueIdentifier<T> {
	private static final HashMap<String, ValueIdentifier<?>> singletons = new HashMap<>();
	
	/**
	 * Returns a globally consistent reference to a {@link ValueIdentifier} with the given ID.
	 *
	 * @param id  The string id of the {@link ValueIdentifier}
	 * @param <T> The type of the value contained within the slots identified by the identifier
	 *
	 * @return The identifier
	 */
	@SuppressWarnings(
		"unchecked"
	)
	public static <T> ValueIdentifier<T> get(String id) {
		if (singletons.containsKey(id)) {
			return (ValueIdentifier<T>) singletons.get(id);
		}
		throw new RuntimeException(format("Cannot find value identifier with id {0}", id));
	}
	
	/**
	 * Returns a globally consistent reference to a {@link ValueIdentifier} with the given ID.
	 * <p>
	 * Creates the value if it doesn't already exist.
	 *
	 * @param id           The string id of the {@link ValueIdentifier}
	 * @param defaultValue The default value to be put in the slots identified by this identifer
	 * @param <T>          The type of the value contained within the slots identified by the identifier
	 *
	 * @return The identifier
	 */
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
	
	/**
	 * @return The default value of slots with this identifier
	 */
	public T getDefaultValue() {
		return defaultValue;
	}
	
	/**
	 * @return The string identifier of this identifier
	 */
	public String getId() {
		return id;
	}
}
