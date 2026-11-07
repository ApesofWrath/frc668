package frc.robot.nodes.pose;

import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

/**
 * Manages gathering pose estimates from a Limelight
 */
public class LimelightNode implements Node {
	/**
	 * Get the value identifier for the limelight pose estimate
	 *
	 * @param id The id of the limelight being polled for estimates
	 *
	 * @return The value identifier for the pose estimate
	 */
	public static ValueIdentifier<LimelightHelpers.PoseEstimate> poseEstimateIdentifier(String id) {
		return ValueIdentifier.get("/vision/" + id + "/pose_estimate", LimelightHelpers.PoseEstimate.class);
	}
	
	private final String id;
	
	/**
	 * Create a limelight node
	 *
	 * @param id The ID of the limelight
	 */
	public LimelightNode(String id) {
		this.id = id;
	}
	
	@Override
	public void configure(NodeInformation information) {
		information.createsOutput(poseEstimateIdentifier(id));
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(id);
		
		update.setValue(poseEstimateIdentifier(id), estimate);
	}
}
