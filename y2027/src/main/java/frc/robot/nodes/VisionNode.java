package frc.robot.nodes;
import static edu.wpi.first.units.Units.Value;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.proto.Wpimath;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import frc.robot.nodes;
import frc.robot.nodes.LimelightHelpers;
import frc.robot.nodes.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.nodes.DrivetrainNode;

public class VisionNode implements Node{
    
    public static final ValueIdentifier<LimelightHelpers.PoseEstimate> VISION_POSE = 
    ValueIdentifier.get("/vision/pose_estimate", null);
    
    
    @Override public void configure(NodeInformation information) {
    information.createsOutput(VISION_POSE);
    }
    
    public void update (NodeUpdateHelper update) {
    
    LimelightHelpers.getBotPoseEstimate_wpiBlue(null)
    
    }

}
