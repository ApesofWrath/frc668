package frc.framework.nodes;

import java.text.MessageFormat;

/**
 * An error that is thrown when a node writes to something without saying so ahead of time
 */
public class UndeclaredWriteException extends RuntimeException {
	private final Node node;
	private final ValueIdentifier<?> identifier;
	
	/**
	 * Create an exception for a write that was not specified ahead of time
	 *
	 * @param node       The node performing the write
	 * @param identifier The target of the write
	 */
	public UndeclaredWriteException(Node node, ValueIdentifier<?> identifier) {
		super(
			MessageFormat.format(
				"The node {0} tried to write {1} without specifying that it does so using the `configure` method. This is undefined behavior, as the framework cannot make sure that nodes that need the value run after this node that sets the value.",
				node.getId(),
				identifier.getId()
			)
		);
		this.node = node;
		this.identifier = identifier;
	}
	
	/**
	 * @return The identifier that was attempted to be written to
	 */
	public ValueIdentifier<?> getIdentifier() {
		return identifier;
	}
	
	/**
	 * @return The node that caused this error
	 */
	public Node getNode() {
		return node;
	}
}
