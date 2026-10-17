package frc.robot.systems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.framework.commonrobot.ControllerState;
import frc.framework.commonrobot.ControllerState.Axis;
import frc.framework.commonrobot.UserInputInformation;
import frc.framework.systems.Priority;
import frc.framework.systems.System;
import frc.framework.systems.SystemInformation;
import frc.framework.systems.SystemUpdateHelper;

import static edu.wpi.first.units.Units.*;

public class HumanDriveSystem implements System {
	
	@Override
	public void configure(SystemInformation information) {
		information.receivesInput(UserInputInformation.CONTROLLER_INPUT);
		information.createsOutput(DrivetrainSubsystem.WANTED_SPEEDS);
	}
	
	@Override
	public void update(SystemUpdateHelper update) {
		ControllerState controllerState = update.getValue(UserInputInformation.CONTROLLER_INPUT).orNull();
		
		if (controllerState == null) {
			update.setValue(DrivetrainSubsystem.WANTED_SPEEDS, new ChassisSpeeds(), Priority.DriverInput); // Don't move if the controller is not connected
			return;
		}
		
		double driveSpeed = FeetPerSecond.of(3).in(MetersPerSecond); // 19.8 feet per second when we feel comfortable
		double rotateSpeed = RotationsPerSecond.of(1).in(RadiansPerSecond); // 2 rotations per second when we feel comfortable
		
		ChassisSpeeds speed = new ChassisSpeeds();
		
		speed.vxMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickX) * driveSpeed;
		speed.vyMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickY) * driveSpeed;
		speed.omegaRadiansPerSecond = controllerState.getAxis(Axis.RightThumbstickX) * rotateSpeed;
		
		update.setValue(DrivetrainSubsystem.WANTED_SPEEDS, speed, Priority.DriverInput);
	}
	
}
