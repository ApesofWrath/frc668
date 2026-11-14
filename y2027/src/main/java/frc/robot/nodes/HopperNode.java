package frc.robot.nodes;

import frc.framework.commonrobot.RobotInformation;
import frc.framework.nodes.NodeInformation;
import frc.framework.nodes.NodeUpdateHelper;
import frc.robot.MotorId;
import frc.robot.constants.Constants.HopperConstants;
import frc.framework.nodes.Node;
import frc.framework.OpMode;
import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.Motors;

import static edu.wpi.first.units.Units.RPM;

public class HopperNode implements Node {
    

    @Override
    public void configure(NodeInformation information){
        information.receivesInput(RobotInformation.OPMODE_VALUE);
    
        information.createsOutput(Motors.controlIdentifier(MotorId.HopperMotorLeft));
        information.createsOutput(Motors.controlIdentifier(MotorId.HopperMotorRight));
    }

    @Override
    public void update(NodeUpdateHelper update){
        
        if(update.getValue(RobotInformation.OPMODE_VALUE).orNull() != OpMode.Teleop){
            return;
        }

        
        update.setValue(Motors.controlIdentifier(MotorId.HopperMotorLeft), MotorControl.velocity(RPM.of(HopperConstants.HOPPER_MOTOR_RPS*HopperConstants.MOTOR_TO_HOPPER_GEAR_RATIO)));
        update.setValue(Motors.controlIdentifier(MotorId.HopperMotorRight), MotorControl.velocity(RPM.of(HopperConstants.HOPPER_MOTOR_RPS*HopperConstants.MOTOR_TO_HOPPER_GEAR_RATIO)));
    }
    
}