package frc.framework.commonrobot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;

/**
 * Handles updating motor information for Phoenix motors
 */
public class PhoenixMotorOutputsNode implements Node {
	private final TalonFX talonFX;
	private final Enum<?> motorId;
	private final PositionVoltage positionControl = new PositionVoltage(0);
	private final VelocityVoltage velocityControl = new VelocityVoltage(0);
	private final VoltageOut voltageControl = new VoltageOut(0);
	
	/**
	 * @param talonFX The motor object to get information from
	 * @param motorId The motor ID for the field identifiers
	 */
	public PhoenixMotorOutputsNode(TalonFX talonFX, Enum<?> motorId) {
		this.talonFX = talonFX;
		this.motorId = motorId;
	}
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(Motors.controlIdentifier(motorId));
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		MotorControl control = update.getValue(Motors.controlIdentifier(motorId)).orNull();
		
		if (control == null) {
			talonFX.setControl(voltageControl.withOutput(0));
			return;
		}
		
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
