package frc.framework;

import java.util.ArrayList;

/**
 * An auto-updating list of nodes of a given type in a node pool.
 */
public class NodeCollection<T> {
	public ArrayList<T> nodes = new ArrayList<>();
	public Class<T> spec;

	public NodeCollection(Class<T> spec) {
		this.spec = spec;
	}

	/**
	 * Internal to framework - Adds a node if it is of the spec type
	 * @param node The node to add
	 */
	@SuppressWarnings("unchecked")
	public void tryAddNode(Object node) {
		if (spec.isInstance(node)) {
			this.nodes.add((T)node);
		}
	}

	/**
	 * Internal to framework - Removes a node if it is in the collection
	 * @param node The node to remove
	 */
	public void tryRemoveNode(Object node) {
		if (nodes.contains(node)) {
			nodes.remove(node);
		}
	}
}
