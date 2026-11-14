package frc.robot.nodes.pose;

import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;


public class OdometryNode implements Node{

    public static ValueIdentifier<SwerveDriveState> ODOMETRY_POSE = ValueIdentifier.get(
        "/drivetrain/estimated_pose",
        SwerveDriveState.class
    );

    public static ValueIdentifier<Rotation2d> IMU_ANGLE = ValueIdentifier.get(
        "/pigeon/rotation",
        Rotation2d.class
    );

    @Override
    public void configure(NodeInformation information){
        information.receivesInput(IMU_ANGLE);
        information.createsOutput(ODOMETRY_POSE);
    }

    SwerveDriveState pose_estimate = SwerveDrivetrain.
    
    @Override
    public void update(NodeUpdateHelper update){
        
        update.setValue();
    }


}
