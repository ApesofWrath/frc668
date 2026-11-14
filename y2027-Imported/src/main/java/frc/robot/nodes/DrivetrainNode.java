package frc.robot.nodes;


import static org.wpilib.units.Units.InchesPerSecond;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import frc.robot.TunerConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;


public class DrivetrainNode implements Node {
	public static ValueIdentifier<ChassisVelocities> WANTED_SPEEDS = ValueIdentifier.get(
		"/drivetrain/wanted_speed",
		ChassisVelocities.class
	);
	
	private final TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(
		TunerConstants.DrivetrainConstants,
		TunerConstants.FrontLeft,
		TunerConstants.FrontRight,
		TunerConstants.BackLeft,
		TunerConstants.BackRight
	);
	
	private final SwerveRequest.ApplyRobotVelocity velocities = new SwerveRequest.ApplyRobotVelocity();
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(WANTED_SPEEDS);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		ChassisVelocities desiredSpeeds = update.getValue(WANTED_SPEEDS).orDefault(new ChassisVelocities());  // not moving is a safe default
		
		double velocityEpsilon = InchesPerSecond.of(1).in(MetersPerSecond);
		double rotationalEpsilon = RotationsPerSecond.of(0.1).in(RadiansPerSecond);
		
		if (MathUtil.isNear(desiredSpeeds.vx, 0, velocityEpsilon) && MathUtil.isNear(
			desiredSpeeds.vy,
			0,
			velocityEpsilon
		) && MathUtil.isNear(desiredSpeeds.omega, 0, rotationalEpsilon)) {
			velocities.Velocity = new ChassisVelocities();
		} else {
			velocities.Velocity = desiredSpeeds;
		}
		drivetrain.setControl(velocities);
	}
}