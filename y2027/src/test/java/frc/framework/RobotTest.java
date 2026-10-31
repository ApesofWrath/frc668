package frc.framework;

import edu.wpi.first.units.measure.Time;
import frc.framework.execution.TimestampedValue;
import frc.framework.logging.LogFrame;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodesManager;
import frc.framework.nodes.Priority;
import frc.framework.nodes.ValueIdentifier;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Milliseconds;

/**
 * A helper class for writing robot tests
 */
public class RobotTest {
	/**
	 * The NodesManager for this test
	 */
	public NodesManager robot = new NodesManager();
	private double desiredTime = 0;
	private int currentTime = 0;
	private int deltaTime = 10;
	
	/**
	 * Add a node to the robot
	 *
	 * @param node The node to add
	 */
	public void addNode(Node node) {
		this.robot.addNode(node);
	}
	
	/**
	 * Advance the simulation by one tick
	 */
	public void advance() {
		LogFrame frame = new LogFrame();
		this.robot.update(this.currentTime, frame);
		this.currentTime += deltaTime;
	}
	
	/**
	 * Advance the simulation by a given amount of time
	 *
	 * @param time The time to advance by
	 */
	public void advance(Time time) {
		this.desiredTime += time.in(Milliseconds);
		while (this.currentTime < this.desiredTime) {
			this.advance();
		}
	}
	
	/**
	 * Get the value stored in a given value identifier
	 *
	 * @param identifier The identifier to read from
	 * @param <T>        The type of the value
	 *
	 * @return The stored value
	 */
	public <T> TimestampedValue<T> get(ValueIdentifier<T> identifier) {
		return robot.getExecutionManager().getValue(identifier.getId());
	}
	
	/**
	 * Set a value on the robot
	 *
	 * @param key   The identifier to set
	 * @param value The value to set it to
	 * @param <T>   The type of the value
	 */
	public <T> void set(ValueIdentifier<T> key, T value) {
		this.robot.getExecutionManager()
			.trySetValue(
				key.getId(),
				new TimestampedValue<>(
					value,
					currentTime + deltaTime,
					Priority.TestingData,
					this.robot.getExecutionManager()
				)
			);
	}
	
	/**
	 * Sets the timestep of the simulated robot
	 *
	 * @param deltaTime The desired time step
	 */
	public void setDeltaTime(int deltaTime) {
		this.deltaTime = deltaTime;
	}
	
	/**
	 * Wait for a predicate to be true, failing after a given timeout
	 *
	 * @param predicate The condition we want to be true
	 * @param timeout   How long we will give before we give up
	 */
	public void waitFor(Supplier<Boolean> predicate, Time timeout) {
		double endTime = desiredTime + timeout.in(Milliseconds);
		
		while (true) {
			if (predicate.get()) { return; }
			
			if (currentTime > endTime) {
				throw new RuntimeException("Waiting for too long.");
			}
			
			advance();
		}
	}
}
