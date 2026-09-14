package frc.framework.systems;

public enum Priority {
	TestingData(6),
	Safety(5),
	Autonomous(4),
	DriverAssistanceOverridesDriver(3),
	DriverInput(2),
	DriverAssistance(1),
	Default(0);
	
	public final int value;
	
	Priority(int value) {
		this.value = value;
	}
}
