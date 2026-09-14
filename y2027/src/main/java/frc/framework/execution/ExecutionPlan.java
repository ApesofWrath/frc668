package frc.framework.execution;

import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;

import java.util.ArrayList;
import java.util.HashMap;

public class ExecutionPlan {
	public ArrayList<String> evaluatedValues = new ArrayList<>();
	public ArrayList<System> systemExecutionOrder = new ArrayList<>();
	public HashMap<System, SystemInformation> systemToInformation = new HashMap<>();
	public ArrayList<System> allSystems = new ArrayList<>();
	public ArrayList<System> systemsBeingPlanned = new ArrayList<>();
	
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
	
	public void build() {
		analyze();
		
		for (System system : allSystems) {
			SystemInformation info = systemToInformation.get(system);
			
			if (info.getOutputIds().isEmpty()) {
				addSystemToExecutionPlan(system, info);
			}
		}
	}
	
	public void debug() {
		java.lang.System.out.println("EXECUTION ORDER:");
		for (System sys : systemExecutionOrder) {
			java.lang.System.out.println("-- " + sys.getId());
		}
	}
}
