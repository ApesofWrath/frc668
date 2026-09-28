package frc.framework.execution;

import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * Represents the evaluation order of various systems
 */
public class ExecutionPlan {
	private final ArrayList<System> allSystems = new ArrayList<>();
	private final ArrayList<System> systemsBeingPlanned = new ArrayList<>();
	private final ArrayList<String> evaluatedValues = new ArrayList<>();
	/**
	 * The order in which systems are to be evaluated
	 */
	public ArrayList<System> systemExecutionOrder = new ArrayList<>();
	/**
	 * A mapping of systems to a reused SystemInformation class configured by the system
	 */
	public HashMap<System, SystemInformation> systemToInformation = new HashMap<>();
	
	/**
	 * Add various systems to be planned
	 *
	 * @param systems The systems to be planned
	 */
	public void addSystems(ArrayList<System> systems) {
		allSystems.addAll(systems);
	}
	
	private void addSystemToExecutionPlan(System system, SystemInformation info) {
		if (systemExecutionOrder.contains(system)) {
			return;
		}
		
		if (systemsBeingPlanned.contains(system)) {
			throw new RuntimeException("System depends on its own outputs: " + system.getId());
		}
		
		systemsBeingPlanned.add(system);
		
		for (String input : info.getInputIds()) {
			addValueToExecutionPlan(input);
		}
		
		systemsBeingPlanned.remove(system);
		
		systemExecutionOrder.add(system);
	}
	
	private void addValueToExecutionPlan(String id) {
		if (evaluatedValues.contains(id)) {
			return;
		}
		
		for (System system : allSystems) {
			SystemInformation info = systemToInformation.get(system);
			
			if (info.getOutputIds().contains(id)) {
				addSystemToExecutionPlan(system, info);
			}
		}
		
		evaluatedValues.add(id);
	}
	
	private void analyze() {
		for (System system : allSystems) {
			SystemInformation information = new SystemInformation();
			
			system.configure(information);
			
			systemToInformation.put(system, information);
		}
	}
	
	/**
	 * Construct an execution order from all the currently stored systems
	 */
	public void build() {
		analyze();
		
		for (System system : allSystems) {
			SystemInformation info = systemToInformation.get(system);
			
			addSystemToExecutionPlan(system, info);
		}
	}
	
	/**
	 * Print the current execution plan to the console for debugging purposes
	 */
	public void debug() {
		java.lang.System.out.println("EXECUTION ORDER:");
		for (System sys : systemExecutionOrder) {
			java.lang.System.out.println("-- " + sys.getId());
		}
	}
}
