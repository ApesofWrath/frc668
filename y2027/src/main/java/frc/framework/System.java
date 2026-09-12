package frc.framework;

public interface System {
    public default String getId() {
        return getClass().getName();
    }
    public void configure(SystemInformation information);
    public void update(SystemUpdateHelper update);
}
