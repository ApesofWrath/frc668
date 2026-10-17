package frc.robot;

import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.Motors;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;

import static edu.wpi.first.units.Units.RPM;

/**
 * Manages intake rollers
 */
public class IntakeNode implements Node {
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
		information.createsOutput(Motors.controlIdentifier(MotorId.IntakeRoller));
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ControllerState state = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		if (state != null && state.getButtonState(ControllerState.Button.A)) {
			update.setValue(
				Motors.controlIdentifier(MotorId.IntakeRoller), // Set the intake motor control
				MotorControl.velocity(RPM.of(500))
			);
		}
	}
}
