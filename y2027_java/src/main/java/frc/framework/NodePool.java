package frc.framework;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Optional;

/**
 * A means by which various parts of the robot can find eachother
 */
public class NodePool {
	public ArrayList<Object> nodes = new ArrayList<>();
	@SuppressWarnings("rawtypes")
	private ArrayList<NodeCollection> collections = new ArrayList<>();
	private ArrayList<Object> hydratedNodes = new ArrayList<>();

	/**
	 * Invoke the hydrate methods and process the hydration attributes on all nodes, effectively linking all the nodes together
	 */
	public void hydrateAll() {
		hydratedNodes.removeIf(it -> !nodes.contains(it));

		for (Object node : nodes) {
			if (!hydratedNodes.contains(node)) {
				hydratedNodes.add(node);
				try {
					hydrateNode(node);
				} catch (IllegalArgumentException | IllegalAccessException e) {
					e.printStackTrace();
				}
			}
		}
	}

	@SuppressWarnings("unchecked")
	private void hydrateNode(Object node) throws IllegalArgumentException, IllegalAccessException {
		if (node instanceof Hydratable hydratableNode) {
			hydratableNode.hydrate(this);
		}

		@SuppressWarnings("rawtypes")
		Class nodeClass = node.getClass();

		for (Field field : nodeClass.getFields()) {
			for (Object annotation : field.getAnnotations()) {
				if (annotation instanceof AutoHydrate autoHydrate) {
					if (field.getType().isAssignableFrom(Optional.class)) {
						field.set(node, locate(autoHydrate.FieldType()));
					} else if (field.getType().isAssignableFrom(NodeCollection.class)) {
						field.set(node, getAll(autoHydrate.FieldType()));
					} else {
						field.set(node, locate(autoHydrate.FieldType()).get());
					}
				}
			}
		}
	}

	/**
	 * Add a node to the node pool, so that it can be found 
	 * @param object The node to be added
	 */
	public void addNode(Object object) {
		nodes.add(object);

		for (@SuppressWarnings("rawtypes") NodeCollection collection : collections) {
			collection.tryAddNode(object);
		}
	}

	/**
	 * Remove a node from the node pool, so that it can no longer be found
	 * @param object The node to be removed
	 */
	public void removeNode(Object object) {
		nodes.remove(object);

		for (@SuppressWarnings("rawtypes") NodeCollection collection : collections) {
			collection.tryRemoveNode(object);
		}
	}

	/**
	 * Find all the nodes of a given class or interface
	 * @param <T> The class or interface to search for
	 * @param spec The reflection type of the class, e.g, `ShooterSubsystem.class`
	 * @return A NodeCollection containing the nodes of the given type, which will be automatically kept in sync
	 */
	@SuppressWarnings({"unchecked", "rawtypes"})
	public <T> NodeCollection<T> getAll(Class<T> spec) {
		for (NodeCollection collection : collections) {
			if (collection.spec == spec) {
				return collection;
			}
		}

		NodeCollection<T> collection = new NodeCollection<>(spec);

		collections.add(collection);

		for (Object node : nodes) {
			collection.tryAddNode(node);
		}

		return collection;
	}

	/**
	 * Find any instance of a node of a given class or interface
	 * @param <T> The class or interface to search for
	 * @param spec The reflection type of the class, e.g, `ShooterSubsystem.class`
	 * @return An Optional containing either nothing, or the node
	 */
	public <T> Optional<T> locate(Class<T> spec) {
		ArrayList<T> nodes = getAll(spec).nodes;

		if (nodes.isEmpty()) {
			return Optional.empty();
		}

		return Optional.of(nodes.get(0));
	}
}
