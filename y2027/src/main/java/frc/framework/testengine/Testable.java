package frc.framework.testengine;

import frc.framework.systems.SystemsManager;
import frc.framework.systems.ValueIdentifier;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.nio.charset.Charset;

/***
 * A given instance of a unit test, with specific parameters
 */
public abstract class Testable {
	/**
	 * The SystemsManager instance
	 */
	public SystemsManager systems = new SystemsManager();
	/**
	 * Information to report about the current test
	 */
	public TestInfo info = new TestInfo();
	/**
	 * A helper for managing assertions
	 */
	public SystemsAssertions check = new SystemsAssertions(systems, info);
	
	/**
	 * The current simulation time (ms)
	 */
	public long currentTime = 0;
	
	/**
	 * The simulation timestep size (ms)
	 */
	public int deltaTime = 10;
	
	/**
	 * User-specific test implementation
	 */
	public abstract void execute();
	
	/**
	 * Get the value in a given slot
	 *
	 * @param ident The slot identifier
	 * @param <T>   The type of the data in the slot
	 *
	 * @return The value
	 */
	public <T> T get(ValueIdentifier<T> ident) {
		return systems.getValue(ident);
	}
	
	/**
	 * Stop setting a value in the simulation
	 *
	 * @param key The slot identifier to unlock
	 */
	public void reset(ValueIdentifier<?> key) {
		systems.testInputs.remove(key.getId());
	}
	
	/**
	 * Execute the test and log information
	 */
	public void run() {
		try {
			execute();
		} catch (Exception e) {
			ByteArrayOutputStream binaryStream = new ByteArrayOutputStream();
			PrintStream stream = new PrintStream(binaryStream);
			
			e.printStackTrace(stream);
			
			info.errors.add(binaryStream.toString(Charset.defaultCharset()));
		}
	}
	
	/**
	 * Lock a given slot to contain a specified value
	 *
	 * @param key   The slot identifier
	 * @param value The data to put in te slot
	 * @param <T>   The type of the data in the slot
	 */
	public <T> void set(ValueIdentifier<T> key, T value) {
		systems.testInputs.put(key.getId(), value);
	}
	
	/**
	 * Advance the simulation by one tick
	 */
	public void update() {
		update(deltaTime);
	}
	
	/**
	 * Advance the simulation by one tick
	 *
	 * @param dt The deltatime to advance by
	 */
	public void update(long dt) {
		systems.update(currentTime);
		currentTime += dt;
	}
}
