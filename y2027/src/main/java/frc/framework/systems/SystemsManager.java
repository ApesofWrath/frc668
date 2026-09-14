package frc.framework.systems;

import frc.framework.execution.ExecutionManager;
import frc.framework.execution.ExecutionPlan;

import java.util.ArrayList;

public class SystemsManager {
	private final ArrayList<System> systems = new ArrayList<>();
	private boolean isExecutionPlanDirty = true;
	private ExecutionPlan executionPlan;
	private final ExecutionManager manager = new ExecutionManager();
	
	public void addSystem(System system) {
		systems.add(system);
		
		isExecutionPlanDirty = true;
	}
	
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
	
	public void update(long time) {
		rebuildExecutionPlan();
		
		manager.plan = executionPlan;
		
		manager.execute(time);
	}
}
