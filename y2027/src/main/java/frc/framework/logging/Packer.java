package frc.framework.logging;

import frc.framework.OpModePacker;
import frc.framework.commonrobot.ControllerStatePacker;

import java.util.ArrayList;

public interface Packer<T> {
	/**
	 * Registry of all packers in existance
	 */
	ArrayList<Packer<?>> packers = new ArrayList<>();
	
	/**
	 * Add a packer to the global registry
	 *
	 * @param packer The packer to add to the registry
	 */
	static void registerPacker(Packer<?> packer) {
		packers.add(packer);
	}
	
	/**
	 * Add all packers to the list of packers
	 */
	static void setupPackers() {
		if (!packers.isEmpty()) { return; }
		
		Packer.registerPacker(new FallbackArrayPacker());
		Packer.registerPacker(new OpModePacker());
		Packer.registerPacker(new ControllerStatePacker());
	}
	
	/**
	 * @return The serialized data type of the packer
	 */
	Class<T> getDataType();
	
	/**
	 * @return A unique identifier for this packer
	 */
	default String getId() {
		return getDataType().getName();
	}
	
	/**
	 * Bundle up values in the log file into a neat object
	 *
	 * @param key    The key to pack
	 * @param helper A utility instance to read unpacked fields from
	 *
	 * @return The packed object
	 */
	T packFields(String key, PackHelper helper);
	
	/**
	 * Store all the internal fields of this type
	 *
	 * @param value      The value to unpack
	 * @param key        The key to unpack it under
	 * @param packHelper A utility instance to write unpacked fields to
	 */
	void unpackFields(T value, String key, PackHelper packHelper);
	
	/**
	 * Store all the internal fields of this type
	 *
	 * @param value      The value to unpack
	 * @param key        The key to unpack it under
	 * @param packHelper A utility instance to write unpacked fields to
	 */
	@SuppressWarnings(
		"unchecked"
	)
	default void unpackFieldsUnsafe(Object value, String key, PackHelper packHelper) {
		unpackFields((T) value, key, packHelper);
	}
}
