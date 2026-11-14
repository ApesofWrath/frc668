import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.mechanisms.swerve;
import com.fasterxml.jackson.annotation.JacksonInject.Value;
import com.fasterxml.jackson.databind.jsontype.impl.AsArrayTypeDeserializer;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

public class FusionNode implements Node {

    
    public 
    
    public static ValueIdentifier<LimelightHelpers.PoseEstimate> POSE_FL = ValueIdentifier.get(
        "/vision/fl/pose_estimate", 
        LimelightHelpers.PoseEstimate
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

    public static ValueIdentifier<Rotation2d.class> IMU_ANGLE = ValueIdentifier.get(
        "/pigeon/rotation",
        Rotation2d.class
    );
    
    
    public static ValueIdentifier<geometry.pose2d> ESTIMATED_POSE = ValueIdentifier.get(
        "drivetrain/pose", geometry.Pose2d
    );


    @Override
    public void configure(NodeInformation information){
        information.receivesInput(POSE_FL);
        information.receivesInput(POSE_UPFL);
        information.receivesInput(POSE_FR);
        information.receivesInput(POSE_UPFR);
        information.receivesInput(IMU_ANGLE);

    }

    
    swerve.addVisionMeasurement()




}