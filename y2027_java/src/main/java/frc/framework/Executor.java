package frc.framework;

/**
 * The execute method will be called every periodic loop, if the Executor is part of a NodePool.
 */
public interface Executor {
    /**
     * Invoked in at the end of every tick
     * @param manager A production manager that can be used to find produced values
     */
    public void execute(ProductionManager manager);
}
