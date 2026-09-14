package frc.framework.systems;

import frc.framework.cache.CacheStrategy;
import frc.framework.cache.NoCacheStrategy;

import java.util.ArrayList;

public class SystemInformation {
	private final ArrayList<String> inputIds = new ArrayList<>();
	private final ArrayList<String> outputIds = new ArrayList<>();
	public CacheStrategy cacheStrategy = new NoCacheStrategy();
	
	public <T> void createsOutput(ValueIdentifier<T> value) {
		outputIds.add(value.getId());
	}
	
	public ArrayList<String> getInputIds() {
		return inputIds;
	}
	
	public ArrayList<String> getOutputIds() {
		return outputIds;
	}
	
	public <T> void recievesInput(ValueIdentifier<T> value) {
		inputIds.add(value.getId());
	}
}
