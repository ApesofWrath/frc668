package frc.framework.commonrobot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.robot.MotorId;

/**
 * Handles updating motor information for Phoenix motors
 */
public class PhoenixMotorOutputsSystem implements System {
	private final TalonFX talonFX;
	private final MotorId motorId;
	private final PositionVoltage positionControl = new PositionVoltage(0);
	private final VelocityVoltage velocityControl = new VelocityVoltage(0);
	private final VoltageOut voltageControl = new VoltageOut(0);
	
	/**
	 * @param talonFX The motor object to get information from
	 * @param motorId The motor ID for the field identifiers
	 */
	public PhoenixMotorOutputsSystem(TalonFX talonFX, MotorId motorId) {
		this.talonFX = talonFX;
		this.motorId = motorId;
	}
	
	@Override
	public void configure(SystemInformation information) {
		information.recievesInput(Motors.controlIdentifier(motorId));
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		MotorControl control = update.getValue(Motors.controlIdentifier(motorId));
		
		switch (control.type()) {
		case Position -> {
			talonFX.setControl(positionControl.withPosition(control.value() / Math.PI / 2));
		}
		case Velocity -> {
			talonFX.setControl(velocityControl.withVelocity(control.value() / Math.PI / 2));
		}
		case Voltage -> {
			talonFX.setControl(voltageControl.withOutput(control.value()));
		}
		}
	}
}
