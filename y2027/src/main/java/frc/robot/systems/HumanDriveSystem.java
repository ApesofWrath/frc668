package frc.robot.systems;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
		
		double driveSpeed = FeetPerSecond.of(3).in(MetersPerSecond); // 19.8 feet per second when we feel comfortable
		double rotateSpeed = RotationsPerSecond.of(1).in(RadiansPerSecond); // 2 rotations per second when we feel comfortable
		
		ChassisSpeeds speed = new ChassisSpeeds();
		
		speed.vxMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickX) * driveSpeed;
		speed.vyMetersPerSecond = controllerState.getAxis(Axis.LeftThumbstickY) * driveSpeed;
		speed.omegaRadiansPerSecond = controllerState.getAxis(Axis.RightThumbstickX) * rotateSpeed;
		
		update.setValue(DrivetrainSubsystem.WANTED_SPEEDS, speed, Priority.DriverInput);
	}
	
}
