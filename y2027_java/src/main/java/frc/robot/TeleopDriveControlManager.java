package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.framework.AutoHydrate;
import frc.framework.Producer;
import frc.framework.Production;
import frc.framework.ProductionManager;
import frc.framework.ProductionPriority;
import frc.framework.controls.Axis;
import frc.framework.controls.Controller;

public class TeleopDriveControlManager implements Producer {
    @AutoHydrate(FieldType = Controller.class)
    private Controller controller;
    @AutoHydrate(FieldType = DriveSubsystem.class)
    private DriveSubsystem drive;

    @Override
    public void produce(ProductionManager manager) {
        // Translation
        float translationTopSpeed =  drive.getDesiredTranslationSpeed();
        Translation2d desiredTranslationSpeed = new Translation2d(controller.getAxis(Axis.LEFT_HORIZONTAL) * translationTopSpeed, controller.getAxis(Axis.LEFT_VERTICAL) * translationTopSpeed);        
        Production translationProduction = new Production(ProductionPriority.Driver);

        translationProduction.set(DriveSubsystem.CHASSIS_TRANSLATION_SPEED_RELROBOT, desiredTranslationSpeed);

        manager.addProduction(translationProduction);


        // Rotation
        float rotationTopSpeed = drive.getDesiredRotationSpeed();
        double desiredRotationSpeed = rotationTopSpeed * controller.getAxis(Axis.RIGHT_HORIZONTAL);
        Production rotationProduction = new Production(ProductionPriority.Driver);

        rotationProduction.set(DriveSubsystem.CHASSIS_ROTATION_SPEED_RELROBOT, desiredRotationSpeed);

        manager.addProduction(rotationProduction);
    }
}
