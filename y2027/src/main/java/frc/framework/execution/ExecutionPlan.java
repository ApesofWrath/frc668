package frc.framework.execution;

import frc.framework.logging.LogFrame;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Represents the evaluation order of various nodes
 */
public class ExecutionPlan {
	private final ArrayList<Node> allNodes = new ArrayList<>();
	private final ArrayList<Node> nodesBeingPlanned = new ArrayList<>();
	private final ArrayList<String> evaluatedValues = new ArrayList<>();
	/**
	 * The order in which nodes are to be evaluated
	 */
	public ArrayList<Node> nodeExecutionOrder = new ArrayList<>();
	/**
	 * A mapping of nodes to a reused nodeInformation class configured by the node
	 */
	public HashMap<Node, NodeInformation> nodeToInformation = new HashMap<>();
	
	/**
	 * Add various nodes to be planned
	 *
	 * @param nodes The nodes to be planned
	 */
	public void addNodes(ArrayList<Node> nodes) {
		allNodes.addAll(nodes);
	}
	
	private void addNodeToExecutionPlan(Node node, NodeInformation info) {
		if (nodeExecutionOrder.contains(node)) {
			return;
		}
		
		if (nodesBeingPlanned.contains(node)) {
			throw new RuntimeException("System depends on its own outputs: " + node.getId());
		}
		
		nodesBeingPlanned.add(node);
		
		for (String input : info.getInputIds()) {
			addValueToExecutionPlan(input);
		}
		
		nodesBeingPlanned.remove(node);
		
		nodeExecutionOrder.add(node);
	}
	
	private void addValueToExecutionPlan(String id) {
		if (evaluatedValues.contains(id)) {
			return;
		}
		
		for (Node node : allNodes) {
			NodeInformation info = nodeToInformation.get(node);
			
			if (info.getOutputIds().contains(id)) {
				addNodeToExecutionPlan(node, info);
			}
		}
		
		evaluatedValues.add(id);
	}
	
	private void analyze() {
		for (Node node : allNodes) {
			NodeInformation information = new NodeInformation();
			
			node.configure(information);
			
			nodeToInformation.put(node, information);
		}
	}
	
	/**
	 * Construct an execution order from all the currently stored nodes
	 */
	public void build() {
		analyze();
		
		for (Node node : allNodes) {
			NodeInformation info = nodeToInformation.get(node);
			
			addNodeToExecutionPlan(node, info);
		}
	}
	
	/**
	 * Print the current execution plan to the console for debugging purposes
	 */
	public void debug() {
		System.out.println("EXECUTION ORDER:");
		for (Node sys : nodeExecutionOrder) {
			System.out.println("-- " + sys.getId());
		}
	}
	
	/**
	 * Write the execution plan to a log frame for observability
	 *
	 * @param frame The frame to store data to
	 */
	public void storeData(LogFrame frame) {
		frame.set("/plan/execution_order", nodeExecutionOrder.stream().map(Node::getId).toArray());
		
		for (Node node : allNodes) {
			String id = node.getId();
			
			frame.set("/plan/" + id + "/outputs", nodeToInformation.get(node).getOutputIds().toArray());
			frame.set("/plan/" + id + "/inputs", nodeToInformation.get(node).getInputIds().toArray());
		}
	}
}
