package frc.framework.replays;

import frc.framework.RobotMaker;
import frc.framework.execution.ExecutionManager;
import frc.framework.execution.TimestampedValue;
import frc.framework.logging.DataLogUtils;
import frc.framework.logging.LogFrame;
import frc.framework.logging.LogReader;
import frc.framework.logging.LogWriter;
import frc.framework.nodes.NodesManager;
import frc.framework.nodes.Priority;
import frc.framework.nodes.ValueIdentifier;

import java.io.File;
import java.io.FileNotFoundException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Objects;
import java.util.Scanner;

/**
 * Manages the replay system
 */
public class ReplayManager {
	/**
	 * Determines if the user wants to do a replay
	 *
	 * @return True if we think the user want to do a replay
	 */
	public static boolean doesUserProbablyWantReplay() {
		return Objects.equals(System.getenv("REPLAY"), "1") || new File("./replay.wpilog").exists();
	}
	
	/**
	 * Find the path of the log file that the user is probably trying to replay
	 *
	 * @return The path to the log file that the user wishes to replay
	 */
	public static Path getReplayPath() {
		ArrayList<String> potentialPaths = new ArrayList<>();
		
		String replayEnvPath = System.getenv("REPLAY");
		if (replayEnvPath != null) {
			potentialPaths.add(replayEnvPath);
		}
		
		potentialPaths.add("./replay.wpilog");
		
		Path akitLogPath = Path.of(System.getProperty("java.io.tmpdir")).resolve("akit-log-path.txt");
		
		if (akitLogPath.toFile().exists()) {
			try (Scanner reader = new Scanner(akitLogPath.toFile())) {
				potentialPaths.add(reader.nextLine());
			} catch (FileNotFoundException e) {
				// We intentionally ignore this exception, as we could resort to other log file locations
			}
		}
		
		for (String path : potentialPaths) {
			Path pathObj = Path.of(path);
			
			if (pathObj.toFile().exists()) {
				return pathObj;
			}
		}
		
		throw new RuntimeException("Cannot determine replay file path.");
	}
	
	private final RobotMaker robotMaker;
	
	/**
	 * Create a replay manager
	 *
	 * @param robotMaker The RobotMaker used to run the replay
	 */
	public ReplayManager(RobotMaker robotMaker) {
		this.robotMaker = robotMaker;
	}
	
	/**
	 * Run and save a replay
	 */
	public void simulateReplay() {
		System.out.println("Running robot in replay mode...");
		Path replayPath = getReplayPath();
		Path outputPath = replayPath.resolveSibling(
			replayPath.getFileName().toString().replace(".wpilog", ".replayed.wpilog")
		);
		NodesManager manager = new NodesManager();
		
		robotMaker.setupFullRobot(manager);
		robotMaker.setupSimulation(manager);
		
		LogReader reader = new LogReader(replayPath.toString());
		LogWriter writer = new LogWriter(outputPath.toString());
		
		while (!reader.isDone()) {
			LogFrame frame = reader.read();
			
			if (frame == null) { break; }
			
			String timestampLocation = DataLogUtils.joinKey("values", "hal", "timestamp");
			Long tickTime = (Long) frame.data.get(timestampLocation);
			
			if (tickTime == null) { continue; }
			
			LogFrame resultingFrame = new LogFrame();
			
			for (ValueIdentifier<?> identifier : ValueIdentifier.getAll()) {
				if (!identifier.isReplayed()) { continue; }
				
				String valueKey = DataLogUtils.joinKey("values", identifier.getId());
				String timestampKey = DataLogUtils.joinKey("values", identifier.getId(), "timestamp");
				
				Object value = frame.data.get(valueKey);
				Long timestamp = (Long) frame.data.get(timestampKey);
				
				if (value == null || timestamp == null) {
					// We can't fully read this field, let's move on.
					continue;
				}
				
				ExecutionManager executionManager = manager.getExecutionManager();
				
				executionManager.trySetValue(
					identifier.getId(),
					new TimestampedValue<>(value, timestamp, Priority.TestingData, executionManager)
				);
			}
			
			manager.update(tickTime, resultingFrame);
			
			frame.data.forEach((key, value) -> {
				resultingFrame.set(DataLogUtils.joinKey("original_data", key), value);
			});
			
			writer.writeFrame(tickTime, resultingFrame);
		}
	}
}
