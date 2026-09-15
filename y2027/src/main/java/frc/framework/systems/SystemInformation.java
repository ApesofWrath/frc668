package frc.framework.systems;

import frc.framework.cache.CacheStrategy;
import frc.framework.cache.NoCacheStrategy;

import java.util.ArrayList;

/**
 * An object used to generate execution plans for systems and decide when to re-evaluate them.
 */
public class SystemInformation {
	private final ArrayList<String> inputIds = new ArrayList<>();
	private final ArrayList<String> outputIds = new ArrayList<>();
	
	/**
	 * The criteria used to determine if this system should be re-evaluated.
	 */
	public CacheStrategy cacheStrategy = new NoCacheStrategy();
	
	/**
	 * Declare that this system produces a given value, and is allowed to set to it when executing. This does not
	 * require that the class always produces the value, but rather producing a value requires that this method is
	 * called when configuring. Attempting to write to an input without invoking this method will result in a runtime
	 * error.
	 *
	 * @param value The value identifier representing the output, in the case of a motor for example, it would be the
	 *              current motor angle.
	 */
	public void createsOutput(ValueIdentifier<?> value) {
		outputIds.add(value.getId());
	}
	
	/**
	 * @return The string identifiers of the inputs of this system
	 */
	public ArrayList<String> getInputIds() {
		return inputIds;
	}
	
	/**
	 * @return The string identifiers of the outputs of this system
	 */
	public ArrayList<String> getOutputIds() {
		return outputIds;
	}
	
	/**
	 * Declare that this system may not be evaluated until the given input has been evaluated. Attempting to read an
	 * input without invoking this method will result in a runtime error.
	 *
	 * @param value The value identifier representing the input, in the case of an automatic alignment system, it may be
	 *              the current robot pose.
	 */
	public void recievesInput(ValueIdentifier<?> value) {
		inputIds.add(value.getId());
	}
}
