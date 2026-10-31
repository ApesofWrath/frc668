package frc.robot.nodes;

import static edu.wpi.first.units.Units.Value;
import edu.wpi.first.math.proto.Wpimath;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;



public class ImuNode implements Node{

	public static ValueIdentifier<Rotation2d> CURRENT_ROTATION = ValueIdentifier.get(
		"/drivetrain/current_rotation",
		Rotation2d.kZero

        
	);
}
    