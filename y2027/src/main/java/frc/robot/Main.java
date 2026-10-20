// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import frc.framework.RobotMaker;
import frc.framework.nodes.NodesRobot;
import frc.framework.replays.ReplayManager;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what you are doing, do
 * not modify this file except to change the parameter class to the startRobot call.
 */
public final class Main {
	/**
	 * Main initialization function. Do not perform any initialization here.
	 *
	 * <p>If you change your main robot class, change the parameter type.
	 *
	 * @param args The command line arguments of the robot program
	 */
	public static void main(String... args) {
		RobotMaker robotMaker = new RobotMaker2026();
		if (ReplayManager.doesUserProbablyWantReplay()) {
			ReplayManager replayManager = new ReplayManager(robotMaker);
			replayManager.simulateReplay();
			return;
		}
		RobotBase.startRobot(() -> new NodesRobot(robotMaker));
	}
	
	private Main() {
	}
}
