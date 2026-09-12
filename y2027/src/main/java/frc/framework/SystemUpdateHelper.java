package frc.framework;

public class SystemUpdateHelper {
    private ExecutionManager executionManager;
    private SystemInformation systemInformation;
    private System system;
    private SystemCachableResult cachableResult;
                
    public SystemUpdateHelper(System system, SystemInformation systemInformation, ExecutionManager executionManager, SystemCachableResult result) {
        this.system = system;
        this.systemInformation = systemInformation;
        this.executionManager = executionManager;
        this.cachableResult = result;
    }

    @SuppressWarnings("unchecked")
    public <T> T getValue(ValueIdentifier<T> value) {
        if (!systemInformation.getInputIds().contains(value.getId())) {
            throw new RuntimeException("The system " + system.getId() + " tried to get the value of " + value.getId() + ", but it never specified ahead of time that it needs it, thus, we cannot be sure that the value has been set.");
        }

        return (T)cachableResult.get(executionManager, value.getId());
    }

    public <T> void setValue(ValueIdentifier<T> identifier, T value, int priority) {
        if (!systemInformation.getOutputIds().contains(identifier.getId())) {
            throw new RuntimeException("The system " + system.getId() + " tried to set the value of " + identifier.getId() + ", but it never specified ahead of time that it sets it, this is throwing an error as it could cause unintended logic bugs.");
        }

        cachableResult.set(identifier.getId(), value, priority);
    }
}
