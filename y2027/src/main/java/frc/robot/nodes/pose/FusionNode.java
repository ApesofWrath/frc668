package frc.robot.nodes.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.fasterxml.jackson.databind.jsontype.impl.AsArrayTypeDeserializer;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

public class FusionNode implements Node {


    public static ValueIdentifier<SwerveDriveState> ODOMETRY_POSE = ValueIdentifier.get(
        "/drivetrain/estimated_pose",
        SwerveDriveState.class
    );


    public static ValueIdentifier<LimelightHelpers.PoseEstimate> POSE_FL = ValueIdentifier.get(
        "/vision/fl/pose_estimate", 
        LimelightHelpers.PoseEstimate.class
    );
    public static ValueIdentifier<LimelightHelpers.PoseEstimate> POSE_FR =  ValueIdentifier.get(
        "/vision/fr/pose_estimate", LimelightHelpers.PoseEstimate.class
    );
    public static ValueIdentifier<LimelightHelpers.PoseEstimate> POSE_UPFR =  ValueIdentifier.get(
        "/vision/upfr/pose_estimate", LimelightHelpers.PoseEstimate.class
    ) ;
    public static ValueIdentifier<LimelightHelpers.PoseEstimate> POSE_UPFL =  ValueIdentifier.get(
        "/vision/upfl/pose_estimate", LimelightHelpers.PoseEstimate.class
    );

    public static ValueIdentifier<Rotation2d> IMU_ANGLE = ValueIdentifier.get(
        "/pigeon/rotation",
        Rotation2d.class
    );
    
    
    public static ValueIdentifier<Pose2d> ESTIMATED_POSE = ValueIdentifier.get(
        "drivetrain/pose", Pose2d.class
    );


    @Override
    public void configure(NodeInformation information){
        information.receivesInput(POSE_FL);
        information.receivesInput(POSE_UPFL);
        information.receivesInput(POSE_FR);
        information.receivesInput(POSE_UPFR);
        information.receivesInput(IMU_ANGLE);
        information.receivesInput(ODOMETRY_POSE);

    }

    
    @Override
    public void update(NodeUpdateHelper update){
        
    }




}