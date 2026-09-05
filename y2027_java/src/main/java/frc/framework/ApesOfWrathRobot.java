package frc.framework;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.framework.controls.XInputController;

/**
 * A helper class that manages the poller, producer, and executor architecture.
 */
public abstract class ApesOfWrathRobot extends TimedRobot {
    public NodePool pool;

    private NodeCollection<DataPoller> pollers;
    private NodeCollection<Producer> producers;
    private NodeCollection<Executor> executors;
    private ProductionManager productionManager;

    private boolean isSetup = false;

    public abstract void setup();

    public void periodic(OpMode mode) {
        if (!isSetup) {
            isSetup = true;
            setup();
        }

        pool.hydrateAll();

        for (DataPoller poller : pollers.nodes) {
            poller.updateData();
        }

        for (Producer producer : producers.nodes) {
            producer.produce(productionManager);
        }
        
        productionManager.resolve();

        for (Executor executor : executors.nodes) {
            executor.execute(productionManager);
        }
    }
    
	@Override
	public void disabledPeriodic() {
        periodic(OpMode.Disabled);
    }

	@Override
	public void autonomousPeriodic() {
        periodic(OpMode.Autonomous);
    }

	@Override
	public void teleopPeriodic() {
        periodic(OpMode.Teleop);
    }

	@Override
	public void testPeriodic() {
        periodic(OpMode.Test);
    }

    public ApesOfWrathRobot() {
        pool = new NodePool();
        pollers = pool.getAll(DataPoller.class);
        producers = pool.getAll(Producer.class);
        executors = pool.getAll(Executor.class);
        productionManager = new ProductionManager();

        pool.addNode(new XInputController(new XboxController(0)));
    }
}
