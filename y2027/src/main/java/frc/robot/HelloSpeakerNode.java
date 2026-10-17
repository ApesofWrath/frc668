package frc.robot;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

/**
 * Prints a hello message directed towards {@link HelloSpeakerNode#NAME_VALUE} to the console
 */
public class HelloSpeakerNode implements Node {
	/**
	 * The name to say hi to
	 */
	public static ValueIdentifier<String> NAME_VALUE = ValueIdentifier.get("/hello/name", String.class);
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(NAME_VALUE);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		System.out.println("Hello, " + update.getValue(NAME_VALUE).orDefault("Default Name"));
	}
}
