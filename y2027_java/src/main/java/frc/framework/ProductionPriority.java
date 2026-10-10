package frc.framework;

public enum ProductionPriority {
    Scoring(3),
    Autonomous(2),
    OverrideDriverAssistance(1),
    Driver(0), // Driver is always zero, because otherwise we have to renumber a lot
    FallbackDriverAssistance(-1);
    
    public final int value;

    private ProductionPriority(int value) {
        this.value = value;
    }
}
