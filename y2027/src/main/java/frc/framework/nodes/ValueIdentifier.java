package frc.framework.nodes;

import java.util.HashMap;
import java.util.InputMismatchException;
import java.util.Set;

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
	 * @param id The string id of the {@link ValueIdentifier}
	 *
	 * @return The identifier
	 */
	public static ValueIdentifier<?> get(String id) {
		if (singletons.containsKey(id)) {
			return singletons.get(id);
		}
		throw new RuntimeException(format("Cannot find value identifier with id {0}", id));
	}
	
	/**
	 * Returns a globally consistent reference to a {@link ValueIdentifier} with the given ID.
	 * <p>
	 * Creates the value if it doesn't already exist.
	 *
	 * @param id   The string id of the {@link ValueIdentifier}
	 * @param type The reflection class of values inside the value identifier
	 * @param <T>  The type of the value contained within the slots identified by the identifier
	 *
	 * @return The identifier
	 */
	@SuppressWarnings(
		"unchecked"
	)
	public static <T> ValueIdentifier<T> get(String id, Class<T> type) {
		if (singletons.containsKey(id)) {
			ValueIdentifier<T> preexisting = (ValueIdentifier<T>) singletons.get(id);
			
			if (!preexisting.getType().equals(type)) {
				throw new InputMismatchException(
					"The existing ValueIdentifier with the ID \"" + id + "\" is of type " + preexisting.getType()
						.getName() + ", but requested the type " + type.getName()
				);
			}
			
			return preexisting;
		}
		ValueIdentifier<T> instance = new ValueIdentifier<>(id, type);
		singletons.put(id, instance);
		return instance;
	}
	
	/**
	 * Returns all value identifiers declared
	 *
	 * @return The set of all declared value identifiers
	 */
	public static Set<ValueIdentifier<?>> getAll() {
		return Set.copyOf(singletons.values());
	}
	
	private final String id;
	
	private final Class<T> type;
	
	private boolean isReplayed = false;
	
	private ValueIdentifier(String id, Class<T> type) {
		this.id = id;
		this.type = type;
	}
	
	/**
	 * @return The string identifier of this identifier
	 */
	public String getId() {
		return id;
	}
	
	/**
	 * @return The type of the value identifier
	 */
	public Class<T> getType() {
		return type;
	}
	
	/**
	 * @return True if this value should be updated from replay files, false otherwise
	 */
	public boolean isReplayed() {
		return isReplayed;
	}
	
	/**
	 * Mark this value identifier as being replayed
	 *
	 * @return This value identifier
	 */
	public ValueIdentifier<T> replayed() {
		this.isReplayed = true;
		return this;
	}
}
