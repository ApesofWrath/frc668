package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import frc.framework.AutoHydrate;
import frc.framework.Producer;
import frc.framework.Production;
import frc.framework.ProductionManager;
import frc.framework.ProductionPriority;
import frc.framework.controls.Button;
import frc.framework.controls.Controller;

public class ShooterControlManager implements Producer {
    @AutoHydrate(FieldType = Controller.class)
    private Controller controller;

    @Override
    public void produce(ProductionManager manager) {
        // if button pressed
            // make a production
            // set desired flyhweel speed on the production
            // add the production to the production manager     
        if (controller.isButtonPressed(Button.A)) {
            Production flywheelSpeedProduction = new Production(ProductionPriority.Driver);

            flywheelSpeedProduction.set(ShooterSubsystem.TARGET_FLYWHEEL_SPEED, RadiansPerSecond.of(1));

            manager.addProduction(flywheelSpeedProduction);
        }
    }
}
