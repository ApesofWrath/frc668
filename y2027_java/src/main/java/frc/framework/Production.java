package frc.framework;

import java.util.HashMap;

/**
 * A collection of values with a given priority.
 * 
 * Productions are atomic, they fail completely or not at all
 * 
 * For example, if there is two productions:
 * <ul>
 * <li>
 * Production A: Contains a translation, priority 1
 * </li>
 * <li>
 * Production B: Contains a translation and a rotation, priority 0
 * </li>
 * </ul>
 * 
 * The ProductionManager, when resolving, will apply Production A first, 
 * however, when trying to apply Production B, it will realize there is 
 * a higher priority translation, and as such, the whole production is 
 * cancelled. Thus, rotation is never set.
 */
public class Production {
	public HashMap<Object, Object> values = new HashMap<Object, Object>();
	public ProductionPriority priority;

	public Production(ProductionPriority priority) {
		this.priority = priority;
	}

	/**
	 * Assign a value to a field reference, note that field references are identified by instance, so two constructed FieldReferences are never the same.
	 * @param <T> The value type of the field
	 * @param reference The field reference, typically a static final field, e.g. ShooterSubsystem.TARGET_FLYWHEEL_SPEED
	 * @param value The value to assign to the field
	 */
	public <T> void set(FieldReference<T> reference, T value) {
		values.put(reference, value);
	}
}
