package frc.framework;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.framework.replays.ReplayManager;
import frc.robot.RobotMaker2026;
import frc.robot.nodes.DrivetrainNode;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.net.URISyntaxException;
import java.nio.file.Path;

public class ReplayTest {
	private final ReplayManager manager = new ReplayManager(new RobotMaker2026());
	
	@Test
	@DisplayName(
		"Drivetrain driving left"
	)
	public void drivetrainDrivingLeft() {
		manager.simulateReplay(pathOfReplay("drivetrain-left"));
		ChassisSpeeds speeds = manager.nodesManager.getExecutionManager()
			.<ChassisSpeeds>getValue(DrivetrainNode.WANTED_SPEEDS.getId())
			.orDefault(new ChassisSpeeds());
		Assertions.assertTrue(speeds.vyMetersPerSecond < 0);
	}
	
	private Path pathOfReplay(String id) {
		try {
			return Path.of(ReplayTest.class.getClassLoader().getResource("replays/" + id + ".wpilog").toURI());
		} catch (URISyntaxException e) {
			throw new RuntimeException(e);
		}
	}
}
