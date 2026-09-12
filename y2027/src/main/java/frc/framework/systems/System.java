package frc.framework.systems;

public interface System {
	default String getId() {
		return getClass().getName();
	}

	void configure(SystemInformation information);

	void update(SystemUpdateHelper update);
}
