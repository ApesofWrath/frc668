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

	@SuppressWarnings("unchecked")
	public void tryAddNode(Object node) {
		if (spec.isInstance(node)) {
			this.nodes.add((T)node);
		}
	}

	public void tryRemoveNode(Object node) {
		if (nodes.contains(node)) {
			nodes.remove(node);
		}
	}
}
