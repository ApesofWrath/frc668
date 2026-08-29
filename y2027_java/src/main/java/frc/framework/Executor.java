package frc.framework;

/**
 * The execute method will be called every periodic loop, if the Executor is part of a NodePool.
 */
public interface Executor {
    public void execute(ProductionManager manager);
}
