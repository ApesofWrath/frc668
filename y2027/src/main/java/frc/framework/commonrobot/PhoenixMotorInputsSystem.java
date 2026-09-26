package frc.framework.commonrobot;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.robot.MotorId;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

/**
 * Handles updating motor information for Phoenix motors
 */
public class PhoenixMotorInputsSystem implements System {
	private final TalonFX talonFX;
	private final MotorId motorId;
	
	/**
	 * @param talonFX The motor object to get information from
	 * @param motorId The motor ID for the field identifiers
	 */
	public PhoenixMotorInputsSystem(TalonFX talonFX, MotorId motorId) {
		this.talonFX = talonFX;
		this.motorId = motorId;
	}
	
	@Override
	public void configure(SystemInformation information) {
		information.createsOutput(Motors.positionIdentifier(motorId));
		information.createsOutput(Motors.velocityIdentifier(motorId));
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		update.setValue(Motors.positionIdentifier(motorId), talonFX.getPosition().getValue().in(Radians));
		update.setValue(Motors.velocityIdentifier(motorId), talonFX.getVelocity().getValue().in(RadiansPerSecond));
	}
}
