package frc.robot;

import frc.framework.ApesOfWrathRobot;

public class Robot extends ApesOfWrathRobot {
	public Robot() {
	}

	@Override
	public void setup() {
		pool.addNode(new HydrationTest());
	}
}
