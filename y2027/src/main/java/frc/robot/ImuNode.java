package frc.robot;

import static edu.wpi.first.units.Units.Value;

import java.nio.channels.Pipe;

import edu.wpi.first.math.proto.Wpimath;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.Priority;
import frc.framework.nodes.ValueIdentifier;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;


public class ImuNode implements Node{

	private Pigeon2 imu = new Pigeon2(22, "swerve");

    
    public static ValueIdentifier<Rotation2d> CURRENT_ROTATION = ValueIdentifier.get(
		"/drivetrain/current_rotation",
		Rotation2d.class
	);

    @Override public void configure(NodeInformation information) 
    {
        information.createsOutput(CURRENT_ROTATION);
    }

    public void update (NodeUpdateHelper update)
    {
        Rotation2d current_rotation = imu.getRotation2d();
        update.setValue(CURRENT_ROTATION, current_rotation, Priority.Default);
    }




}
    