package frc.robot;

import frc.framework.cache.MemoizationCacheStrategy;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

/**
 * Extends a name {@link NameExtenderNode#NAME_TO_EXTEND_VALUE} to Name Nameington, and outputs to
 * {@link HelloSpeakerNode#NAME_VALUE}
 */
public class NameExtenderNode implements Node {
	/**
	 * The name to extend
	 */
	public static ValueIdentifier<String> NAME_TO_EXTEND_VALUE = ValueIdentifier.get(
		"/name_extender/name_to_extend",
		String.class
	);
	
	@Override
	public void configure(NodeInformation information) {
		information.cacheStrategy = new MemoizationCacheStrategy();
		information.receivesInput(NAME_TO_EXTEND_VALUE);
		information.createsOutput(HelloSpeakerNode.NAME_VALUE);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		System.out.println("extending name");
		String name = update.getValue(NAME_TO_EXTEND_VALUE).orDefault("Default Name");
		update.setValue(HelloSpeakerNode.NAME_VALUE, name + " " + name + "ington");
	}
}
