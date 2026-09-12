package frc.framework;

import java.util.ArrayList;

import frc.framework.cache.CacheStrategy;
import frc.framework.cache.NoCacheStrategy;

public class SystemInformation {
    private ArrayList<String> inputIds = new ArrayList<>();
    private ArrayList<String> outputIds = new ArrayList<>();
    public CacheStrategy cacheStrategy = new NoCacheStrategy();

    /**
     * This exists so that the SystemsManager can find the values and determine their defaults.
     */
    @SuppressWarnings("rawtypes")
    public ArrayList<ValueIdentifier> values = new ArrayList<>();

    public ArrayList<String> getInputIds() {
        return inputIds;
    }

    public ArrayList<String> getOutputIds() {
        return outputIds;
    }

    public <T> void recievesInput(ValueIdentifier<T> value) {
        inputIds.add(value.getId());
        values.add(value);
    }

    public <T> void createsOutput(ValueIdentifier<T> value) {
        outputIds.add(value.getId());
        values.add(value);
    }
}
