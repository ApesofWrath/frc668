package frc.robot.nodes;


import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
		ChassisSpeeds desiredSpeeds = update.getValue(WANTED_SPEEDS).orDefault(new ChassisSpeeds());  // not moving is a safe default
		
		double velocityEpsilon = InchesPerSecond.of(1).in(MetersPerSecond);
		double rotationalEpsilon = RotationsPerSecond.of(0.1).in(RadiansPerSecond);
		
		if (MathUtil.isNear(desiredSpeeds.vxMetersPerSecond, 0, velocityEpsilon) && MathUtil.isNear(
			desiredSpeeds.vyMetersPerSecond,
			0,
			velocityEpsilon
		) && MathUtil.isNear(desiredSpeeds.omegaRadiansPerSecond, 0, rotationalEpsilon)) {
			speeds.Speeds = new ChassisSpeeds();
			drivetrain.setControl(speeds);
		} else {
			speeds.Speeds = desiredSpeeds;
			drivetrain.setControl(speeds);
		}
	}
}