package frc.robot.systems;

import frc.framework.commonrobot.RobotInformation;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;
import edu.wpi.first.wpilibj.DriverStation;

public class IntakeSubsystem implements System {

    boolean IntakeTeleopInit = DriverStation.isEnabled();
    boolean IntakeDeployed = false;

    @Override
    public void configure(SystemInformation information) {
        
        if (IntakeTeleopInit){

        }
    }

    @Override
    public void update(SystemUpdateHelper update) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'update'");
    }

}