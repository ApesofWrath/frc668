package frc.framework.nodes;

import java.text.MessageFormat;

/**
 * An exception for when a node reads something without specifying so ahead of time
 */
public class UndeclaredReadException extends RuntimeException {
	private final Node node;
	private final ValueIdentifier<?> identifier;
	
	/**
	 * Create an exception for a node that reads something without specifying that it needs the value
	 *
	 * @param node       The node performing the read
	 * @param identifier The identifier that the node tried to read from
	 */
	public UndeclaredReadException(Node node, ValueIdentifier<?> identifier) {
		super(
			MessageFormat.format(
				"The node {0} tried to read {1} without specifying that it needs it using the `configure` method. The framework cannot guarantee that this value has been evaluated unless you specify that it must be evaluated before this system is run.",
				node.getId(),
				identifier.getId()
			)
		);
		this.node = node;
		this.identifier = identifier;
	}
	
	/**
	 *
	 * @return The identifier that was attempted to be read from
	 */
	public ValueIdentifier<?> getIdentifier() {
		return identifier;
	}
	
	/**
	 *
	 * @return The node that tried to perform the read
	 */
	public Node getNode() {
		return node;
	}
}
