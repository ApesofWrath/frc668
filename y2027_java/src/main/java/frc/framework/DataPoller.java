package frc.framework;

/**
 * The updateData method will be called every periodic loop, if the DataPoller is part of a {@link frc.framework.NodePool}, it is called before any producers are invoked.
 */
public interface DataPoller {
    /**
     * Update fields pertaining to data on the component, such that Producers can access it,
     * As an example, if you had a field on your class called flywheelSpeed, you would update it in this method
     */
    public void updateData();
}
