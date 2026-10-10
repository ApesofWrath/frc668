package frc.robot.systems;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import frc.robot.TunerConstants;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;
import frc.framework.systems.*;

public class DrivetrainSubsystem implements System {
	public static ValueIdentifier<ChassisSpeeds> WANTED_SPEEDS = ValueIdentifier.get(
		"/drivetrain/wanted_speed",
		new ChassisSpeeds()
	);
	
	private TunerSwerveDrivetrain drivetrain = new TunerSwerveDrivetrain(
		TunerConstants.DrivetrainConstants,
		TunerConstants.FrontLeft,
		TunerConstants.FrontRight,
		TunerConstants.BackLeft,
		TunerConstants.BackRight
	);
	
	private SwerveRequest.ApplyRobotSpeeds speeds = new SwerveRequest.ApplyRobotSpeeds();
	
	@Override
	public void configure(SystemInformation information) {
		information.recievesInput(WANTED_SPEEDS);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		speeds.Speeds = update.getValue(WANTED_SPEEDS);
		
		drivetrain.setControl(speeds);
	}
}