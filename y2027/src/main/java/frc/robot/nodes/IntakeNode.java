package frc.robot.nodes;



import frc.framework.OpMode;
import frc.framework.cache.MemoizationCacheStrategy;
import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.Motors;
import frc.framework.commonrobot.RobotInformation;
import frc.framework.nodes.Node;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.framework.nodes.ValueIdentifier;
import frc.robot.MotorId;
import frc.robot.constants.Constants;

import static edu.wpi.first.units.Units.RPM;

public class IntakeNode implements Node {
    public static ValueIdentifier<Boolean> REQUEST_SPIN_INTAKE = ValueIdentifier.get(
		"/drivetrain/request_spin_intake",
		Boolean.class
	);
    
    @Override
    public void configure(NodeInformation information){
        
        information.cacheStrategy = new MemoizationCacheStrategy();
        
        information.receivesInput(RobotInformation.OPMODE_VALUE);
        information.receivesInput(REQUEST_SPIN_INTAKE);

        information.createsOutput(Motors.controlIdentifier(MotorId.IntakeRollerTop));
        information.createsOutput(Motors.controlIdentifier(MotorId.IntakeRollerBottom));
    }


    @Override
    public void update(NodeUpdateHelper update){
        OpMode opMode = update.getValue(RobotInformation.OPMODE_VALUE).orNull();
        if (opMode != OpMode.Teleop){
			return;
		}

        boolean shouldSpin = update.getValue(REQUEST_SPIN_INTAKE).orDefault(true, 999999999);

        if(shouldSpin){
            update.setValue(Motors.controlIdentifier(MotorId.IntakeRollerTop),
                            MotorControl.velocity(RPM.of(Constants.IntakeConstants.ACTIVE_ROLLER_SPEED_RPS)));
            update.setValue(Motors.controlIdentifier(MotorId.IntakeRollerBottom), 
                            MotorControl.velocity(RPM.of(Constants.IntakeConstants.ACTIVE_ROLLER_SPEED_RPS)));
        }
        else{
            update.setValue(Motors.controlIdentifier(MotorId.IntakeRollerTop), MotorControl.velocity(0));
            update.setValue(Motors.controlIdentifier(MotorId.IntakeRollerBottom), MotorControl.velocity(0));
        }
        
        

    }
}
