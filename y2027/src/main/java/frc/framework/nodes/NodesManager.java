package frc.framework.nodes;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.ExecutionPlan;
import frc.framework.logging.LogFrame;

import java.util.ArrayList;

/**
 * An object that delegates the planning and execution of a value graph, represented by various nodes.
 */
public class NodesManager {
	private final ArrayList<Node> nodes = new ArrayList<>();
	private final ExecutionManager manager = new ExecutionManager();
	private boolean isExecutionPlanDirty = true;
	private ExecutionPlan executionPlan;
	
	/**
	 * Add a given node to the value graph
	 *
	 * @param node The node that should be added to the value graph
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
	 * Posts the data evaluated by various nodes to NetworkTables, which is useful for debugging.
	 */
	public void publishValuesToNetworkTables() {
		manager.publishValuesToNetworkTables();
	}
	
	/**
	 * Evaluate the value graph, and rebuild it if need be.
	 *
	 * @param time  The current time, which is passed to {@link frc.framework.cache.CacheStrategy}s to determine if
	 *              nodes
	 *              should be re-run.
	 * @param frame The logging frame used to capture the behavior of the robot at any given time
	 */
	public void update(long time, LogFrame frame) {
		maybeRebuildExecutionPlan();
		
		executionPlan.storeData(frame);
		
		manager.execute(time, frame, executionPlan);
	}
}
