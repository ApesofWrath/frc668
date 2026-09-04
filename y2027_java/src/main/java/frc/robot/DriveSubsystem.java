package frc.robot;


import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.framework.DataPoller;
import frc.framework.Executor;
import frc.framework.FieldReference;
import frc.framework.ProductionManager;
import frc.robot.TunerConstants.TunerSwerveDrivetrain;

public class DriveSubsystem implements DataPoller, Executor {
    public static final FieldReference<Translation2d> CHASSIS_TRANSLATION_SPEED_RELROBOT = new FieldReference<Translation2d>(Translation2d.kZero);
    public static final FieldReference<Double> CHASSIS_ROTATION_SPEED_RELROBOT = new FieldReference<Double>(0.0);

    public ChassisSpeeds currentChassisSpeedsRelRobot = new ChassisSpeeds();

    private TunerSwerveDrivetrain drivetrain = new TunerConstants.TunerSwerveDrivetrain(TunerConstants.DrivetrainConstants, TunerConstants.FrontLeft, TunerConstants.FrontRight, TunerConstants.BackLeft, TunerConstants.BackRight);
    private SwerveRequest.ApplyRobotSpeeds robotSpeedRequest = new SwerveRequest.ApplyRobotSpeeds();

    public float getDesiredTranslationSpeed() {
        return 15f; // 15 meters per second, shhh, this is just example code
    }

    public float getDesiredRotationSpeed() {
        return 3.14f; // 1 rotation per second, again, example code
    }

    @Override
    public void execute(ProductionManager manager) {
        Translation2d translationSpeed = manager.getValue(CHASSIS_TRANSLATION_SPEED_RELROBOT);
        double rotationSpeed = manager.getValue(CHASSIS_ROTATION_SPEED_RELROBOT);

        ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds(translationSpeed.getX(), translationSpeed.getY(), rotationSpeed);

        robotSpeedRequest.withSpeeds(desiredChassisSpeeds);

        drivetrain.setControl(robotSpeedRequest);
    }

    @Override
    public void updateData() {
        currentChassisSpeedsRelRobot = ChassisSpeeds.fromRobotRelativeSpeeds(drivetrain.getState().Speeds, null);
    }
}
