package frc.framework.systems;

public interface System {
	void configure(SystemInformation information);
	
	default String getId() {
		return getClass().getName();
	}
	
	void update(SystemUpdateHelper update);
}
