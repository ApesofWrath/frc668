package frc.framework.systems;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.ExecutionPlan;

import java.util.ArrayList;

/**
 * An object that delegates the planning and execution of a value graph, represented by various systems.
 */
public class SystemsManager {
	private final ArrayList<System> systems = new ArrayList<>();
	private final ExecutionManager manager = new ExecutionManager();
	private boolean isExecutionPlanDirty = true;
	private ExecutionPlan executionPlan;
	
	/**
	 * Add a given system to the value graph
	 *
	 * @param system The system that should be added to the value graph
	 */
	public void addSystem(System system) {
		systems.add(system);
		
		isExecutionPlanDirty = true;
	}
	
	/**
	 * Posts the data evaluated by various systems to NetworkTables, which is useful for debugging.
	 */
	public void publishValuesToNetworkTables() {
		manager.publishValuesToNetworkTables();
	}
	
	private void rebuildExecutionPlan() {
		if (!isExecutionPlanDirty) {
			return;
		}
		
		isExecutionPlanDirty = false;
		
		executionPlan = new ExecutionPlan();
		
		executionPlan.addSystems(systems);
		executionPlan.build();
	}
	
	/**
	 * Evaluate the value graph, and rebuild it if need be.
	 *
	 * @param time The current time, which is passed to {@link frc.framework.cache.CacheStrategy}s to determine if
	 *             systems should be re-run.
	 */
	public void update(long time) {
		rebuildExecutionPlan();
		
		manager.plan = executionPlan;
		
		manager.execute(time);
	}
}
