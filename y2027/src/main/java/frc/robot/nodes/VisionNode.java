package frc.robot.nodes;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import frc.robot.nodes;
import frc.robot.nodes.LimelightHelpers;
import frc.robot.nodes.LimelightHelpers.LimelightResults;
import frc.robot.nodes.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.nodes.DrivetrainNode;


public class VisionNode implements Node{
    
    
    
    static  
    
    public static final ValueIdentifier<LimelightHelpers.PoseEstimate> VISION_POSE = 
    ValueIdentifier.get("/vision/pose_estimate", null);

    public static ValueIdentifier<Rotation2d> CURRENT_ROTATION = 
    ValueIdentifier.get("/drivetrain/current_rotation", Rotation2d.class);
    
    
    @Override public void configure(NodeInformation information) {
    information.recivesInput(CURRENT_ROTATION);
    information.createsOutput(VISION_POSE);
    }
    
    public void update (NodeUpdateHelper update) 
    {
        PoseEstimate pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("upfl");
        NetworkTableInstance.getDefault().getTable(null).getEntry(null).getDouble(0);
    
    }

}
