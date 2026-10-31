package frc.robot.nodes;

import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveModule;
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
	
	private final StaticBrake brakeControl = new StaticBrake();
	
	private final SwerveRequest.ApplyRobotSpeeds speeds = new SwerveRequest.ApplyRobotSpeeds();
	
	@Override
	public void configure(NodeInformation information) {
		information.receivesInput(WANTED_SPEEDS);
	}
	
	@Override
	public void update(NodeUpdateHelper update) {
		speeds.Speeds = update.getValue(WANTED_SPEEDS).orDefault(new ChassisSpeeds()); // not moving is a safe default
		
		double velocityEpsilon = InchesPerSecond.of(1).baseUnitMagnitude();
		double rotationalEpsilon = RotationsPerSecond.of(0.1).baseUnitMagnitude();
		
		if (MathUtil.isNear(speeds.Speeds.vxMetersPerSecond, 0, velocityEpsilon) && MathUtil.isNear(
			speeds.Speeds.vyMetersPerSecond,
			0,
			velocityEpsilon
		) && MathUtil.isNear(speeds.Speeds.omegaRadiansPerSecond, 0, rotationalEpsilon)) {
			for (SwerveModule<TalonFX, TalonFX, CANcoder> module : drivetrain.getModules()) {
				TalonFX steer = module.getSteerMotor();
				TalonFX drive = module.getDriveMotor();
				
				steer.setControl(brakeControl);
				drive.setControl(brakeControl);
			}
		} else {
			drivetrain.setControl(speeds);
		}
	}
}