package frc.framework.nodes;

/**
 * A node in the evaluation graph. A system takes in inputs and returns outputs.
 *
 * @see #configure
 * @see #update
 */
public interface Node {
	/**
	 * Inform the evaluation planning engine of what your system is going to do.
	 *
	 * @param information An object with methods of providing the information about your system, most notably, the
	 *                    {@link NodeInformation#createsOutput} and {@link NodeInformation#receivesInput} methods.
	 *                    Additionally, there is also the {@link NodeInformation#cacheStrategy} field, which can be set
	 *                    to determine when your system is evaluated, which may be useful in performance-critical
	 *                    contexts.
	 */
	void configure(NodeInformation information);
	
	/**
	 * An identifier uniquely distinguishing this system from others in log files, etc.
	 *
	 * @return A unique identifier, by default, this returns the fullly qualified name of your class, e.g
	 *         <code>frc.framework.System</code>
	 */
	default String getId() {
		return getClass().getName();
	}
	
	/**
	 * Take in the inputs of your system, and create various outputs, or, in the case of physical hardware, apply
	 * changes based on the inputs.
	 *
	 * @param update An object with {@link NodeUpdateHelper#getValue} and {@link NodeUpdateHelper#setValue} methods,
	 *               allowing you to take in inputs and create outputs for your class. Please note that if you read or
	 *               write a value that was not explicitly declared in {@link Node#configure}, the methods will throw an
	 *               exception.
	 */
	void update(NodeUpdateHelper update);
}
