package frc.framework;

/**
 * Identifies a value which can be produced and consumed by various nodes.
 */
public class FieldReference<T> {
	public T defaultValue;

	public FieldReference(T defaultValue) {
		this.defaultValue = defaultValue;
	}
}
