package frc.robot.nodes;

import edu.wpi.first.math.proto.Wpimath;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import frc.robot.TunerConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;

public class DrivetrainNode implements Node {
	public static ValueIdentifier<ChassisSpeeds> WANTED_SPEEDS = ValueIdentifier.get(
		"/drivetrain/wanted_speed",
		ChassisSpeeds.class
	);
	
	private final TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(
		TunerConstants.DrivetrainConstants,
		TunerConstants.FrontLeft,
		TunerConstants.FrontRight,
		TunerConstants.BackLeft,
		TunerConstants.BackRight
	);
	
	private final SwerveRequest.ApplyRobotSpeeds speeds = new SwerveRequest.ApplyRobotSpeeds();
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(WANTED_SPEEDS);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		speeds.Speeds = update.getValue(WANTED_SPEEDS).orDefault(new ChassisSpeeds()); // not moving is a safe default

		
		drivetrain.setControl(speeds);
	}
}