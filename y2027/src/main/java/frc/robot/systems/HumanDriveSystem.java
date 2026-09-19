package frc.robot.systems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.commonrobot.ControllerState.Axis;
import frc.framework.systems.Priority;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

public class HumanDriveSystem implements System {
	
	@Override
	public void configure(SystemInformation information) {
		information.recievesInput(UserInputInformation.CONTROLLER_INPUT);
		information.createsOutput(DrivetrainSubsystem.WANTED_SPEEDS);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT);
		
		
		ChassisSpeeds speed = new ChassisSpeeds();
		
		speed.vxMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickX);
		speed.vyMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickY);
		speed.omegaRadiansPerSecond = controllerState.getAxis(Axis.RightThumbstickX);
		
		update.setValue(DrivetrainSubsystem.WANTED_SPEEDS, speed, Priority.DriverInput);
	}
	
}
