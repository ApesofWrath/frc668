package frc.framework;

/**
 * The updateData method will be called every periodic loop, if the DataPoller is part of a {@link frc.framework.NodePool}, it is called before any producers are invoked.
 */
public interface DataPoller {
    public void updateData();
}
