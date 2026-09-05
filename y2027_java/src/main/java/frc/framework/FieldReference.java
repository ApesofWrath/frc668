package frc.framework;

/**
 * Identifies a value which can be produced and consumed by various nodes.
 * 
 * This is a key in many things, and reference matters, e.g, if one constructs two FieldReferences, setting them on a Production does not conflict
 */
public class FieldReference<T> {
	public T defaultValue;

	public FieldReference(T defaultValue) {
		this.defaultValue = defaultValue;
	}
}
