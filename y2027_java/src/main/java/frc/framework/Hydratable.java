package frc.framework;

/**
 * If a Hydratable is added to a NodePool, the hydrate method will be called, and can be used to wire up references to other nodes.
 */
public interface Hydratable {
    public void hydrate(NodePool pool);
}
