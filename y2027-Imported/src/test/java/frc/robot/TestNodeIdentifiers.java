package frc.robot;

import frc.framework.nodes.ValueIdentifier;

public class TestNodeIdentifiers {
	public static final ValueIdentifier<Integer> alpha = ValueIdentifier.get("/test/alpha", Integer.class);
	public static final ValueIdentifier<Integer> beta = ValueIdentifier.get("/test/beta", Integer.class);
	public static final ValueIdentifier<Integer> gamma = ValueIdentifier.get("/test/gamma", Integer.class);
	public static final ValueIdentifier<Integer> delta = ValueIdentifier.get("/test/delta", Integer.class);
}
