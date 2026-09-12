package frc.framework;

import java.util.ArrayList;
import java.util.HashMap;

public class SystemsManager {
    private ArrayList<System> systems = new ArrayList<>();
    private boolean isExecutionPlanDirty = false;
    private ExecutionPlan executionPlan;
    private ExecutionManager manager = new ExecutionManager();

    public void addSystem(System system) {
        systems.add(system);

        isExecutionPlanDirty = true;
    }

    private void rebuildExecutionPlan() {
        if (!isExecutionPlanDirty) return;

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
