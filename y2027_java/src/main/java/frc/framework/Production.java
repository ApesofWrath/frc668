package frc.framework;

import java.util.HashMap;

public class Production {
	public HashMap<Object, Object> values = new HashMap<Object, Object>();
	public ProductionPriority priority;

	public Production(ProductionPriority priority) {
		this.priority = priority;
	}

	public <T> void set(FieldReference<T> reference, T value) {
		values.put(reference, value);
	}
}
