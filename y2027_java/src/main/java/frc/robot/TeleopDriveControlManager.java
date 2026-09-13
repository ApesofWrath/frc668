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

        double LEFT_HORIZONTAL = controller.getAxis(Axis.LEFT_HORIZONTAL);
        double LEFT_VERTICAL = controller.getAxis(Axis.LEFT_VERTICAL);
        float translationTopSpeed =  drive.getDesiredTranslationSpeed();
    
        Translation2d desiredTranslationSpeed = new Translation2d(translationTopSpeed * Math.abs(LEFT_HORIZONTAL) > 0.05 ? LEFT_HORIZONTAL : 0 , 
                                                        translationTopSpeed  * Math.abs(LEFT_VERTICAL) > 0.05 ? LEFT_VERTICAL : 0 );        
            
        Production translationProduction = new Production(ProductionPriority.Driver);
        translationProduction.set(DriveSubsystem.CHASSIS_TRANSLATION_SPEED_RELROBOT, desiredTranslationSpeed);
        manager.addProduction(translationProduction);


        // Rotation
        float rotationTopSpeed = drive.getDesiredRotationSpeed();
        
        double RIGHT_HORIZONTAL = controller.getAxis(Axis.RIGHT_HORIZONTAL);

        double desiredRotationSpeed = rotationTopSpeed * (Math.abs(RIGHT_HORIZONTAL) > 0.05 ? RIGHT_HORIZONTAL: 0);
        Production rotationProduction = new Production(ProductionPriority.Driver);

        rotationProduction.set(DriveSubsystem.CHASSIS_ROTATION_SPEED_RELROBOT, desiredRotationSpeed);

        manager.addProduction(rotationProduction);
    }
}
