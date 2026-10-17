package frc.robot.nodes;

import edu.wpi.first.wpilibj.DriverStation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;

public class IntakeNode implements Node {
	
	boolean IntakeTeleopInit = DriverStation.isEnabled();
	boolean IntakeDeployed = false;
	
	@Override
	public void configure(NodeInformation information) {
		
		if (IntakeTeleopInit) {
			
		}
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'update'");
	}
	
}