package frc.robot;

import frc.framework.ApesOfWrathRobot;

public class Robot extends ApesOfWrathRobot {
	public Robot() {
	}

	@Override
	public void setup() {
		pool.addNode(new DriveSubsystem());
		pool.addNode(new TeleopDriveControlManager());
		pool.addNode(new ShooterSubsystem());
		pool.addNode(new ManualController());
	}
}
