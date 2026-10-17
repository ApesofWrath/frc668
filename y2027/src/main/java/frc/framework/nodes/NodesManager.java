package frc.framework.nodes;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.ExecutionPlan;

import java.util.ArrayList;

/**
 * An object that delegates the planning and execution of a value graph, represented by various systems.
 */
public class NodesManager {
	private final ArrayList<Node> nodes = new ArrayList<>();
	private final ExecutionManager manager = new ExecutionManager();
	private boolean isExecutionPlanDirty = true;
	private ExecutionPlan executionPlan;
	
	/**
	 * Add a given system to the value graph
	 *
	 * @param node The system that should be added to the value graph
	 */
	public void addNode(Node node) {
		nodes.add(node);
		
		isExecutionPlanDirty = true;
	}
	
	private void maybeRebuildExecutionPlan() {
		if (!isExecutionPlanDirty) {
			return;
		}
		
		isExecutionPlanDirty = false;
		
		executionPlan = new ExecutionPlan();
		
		executionPlan.addNodes(nodes);
		executionPlan.build();
	}
	
	/**
	 * Posts the data evaluated by various systems to NetworkTables, which is useful for debugging.
	 */
	public void publishValuesToNetworkTables() {
		manager.publishValuesToNetworkTables();
	}
	
	/**
	 * Evaluate the value graph, and rebuild it if need be.
	 *
	 * @param time The current time, which is passed to {@link frc.framework.cache.CacheStrategy}s to determine if
	 *             systems should be re-run.
	 */
	public void update(long time) {
		maybeRebuildExecutionPlan();
		
		manager.execute(time, executionPlan);
	}
}
