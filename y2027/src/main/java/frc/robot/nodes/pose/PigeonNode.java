package frc.robot.nodes.pose;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;

/**
 * Manages gathering and publishing data from the Pigeon
 */
public class PigeonNode implements Node {
	/**
	 * The current rotation of the Pigeon
	 */
	public static final ValueIdentifier<Rotation2d> ROTATION = ValueIdentifier.get(
		"/pigeon/rotation",
		Rotation2d.class
	);
	private final Pigeon2 pigeon;
	
	/**
	 * Create an IMU node
	 *
	 * @param canID  The CAN ID of the Pigeon
	 * @param canBus The CAN bus of the Pigeon
	 */
	public PigeonNode(int canID, CANBus canBus) {
		pigeon = new Pigeon2(canID, canBus);
	}
	
	
	@Override
	public void configure(NodeInformation information) {
		information.createsOutput(ROTATION);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		update.setValue(ROTATION, pigeon.getRotation2d());
	}
}
