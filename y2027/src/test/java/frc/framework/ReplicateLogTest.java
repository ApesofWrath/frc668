package frc.framework;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import frc.framework.commonrobot.ControllerState;
import frc.framework.logging.DataLogUtils;
import frc.framework.logging.LogFrame;
import frc.framework.logging.LogReader;
import frc.framework.logging.LogWriter;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ReplicateLogTest {
	@Test
	@DisplayName(
		"Replicate a log file"
	)
	public void replicateLog() throws Exception {
		LogFrame alpha = new LogFrame();
		LogFrame beta = new LogFrame();
		LogFrame gamma = new LogFrame();
		
		ChassisSpeeds chassisSpeeds = new ChassisSpeeds(5, 5, 5);
		
		alpha.set("/abc", 12);
		beta.set("/cc", chassisSpeeds);
		
		ArrayList<LogFrame> frames = new ArrayList<>();
		
		frames.add(alpha);
		frames.add(beta);
		frames.add(gamma);
		
		tryReplicateLog(frames);
	}
	
	@Test
	@DisplayName(
		"Replicate 1000 random log files"
	)
	public void tortureTest() throws Exception {
		// Consistent seed
		Random rng = new Random(668);
		ControllerState testState = new ControllerState();
		testState.setButtonState(ControllerState.Button.A, true);
		testState.setAxis(ControllerState.Axis.LeftThumbstickX, -11);
		testState.setAxis(ControllerState.Axis.RightThumbstickX, Float.MAX_VALUE);
		testState.setAxis(ControllerState.Axis.RightThumbstickY, Double.MAX_VALUE);
		Object[] values = new Object[]{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 3.14, new ChassisSpeeds(
			5,
			5,
			5
		), null, "ADjlkJLKASJDLKAMSDKLMALKSDMLKQMWE", new ControllerState(), testState};
		String[] keys = new String[]{"alpha", "/beta", "/gam//me", "/delta", "/foo", "/bar", "/", "/-", "/buzz", "    ", "asdkljasdl", ")(*#)", "JJdas/d/asd/asdasdljkasdkkm"};
		
		for (int i = 0; i < 1000; i++) {
			ArrayList<LogFrame> frames = new ArrayList<>();
			
			for (int j = 0; j < 100; j++) {
				LogFrame frame = new LogFrame();
				
				for (int k = 0; k < 10; k++) {
					frame.set(DataLogUtils.joinKey(keys[rng.nextInt(keys.length)]), values[rng.nextInt(values.length)]);
				}
				
				frames.add(frame);
			}
			
			tryReplicateLog(frames);
		}
	}
	
	public void tryReplicateLog(ArrayList<LogFrame> frames) throws IOException {
		
		ByteArrayOutputStream byteWriter = new ByteArrayOutputStream();
		
		try (LogWriter writer = new LogWriter(byteWriter)) {
			int i = 0;
			for (LogFrame frame : frames) {
				writer.writeFrame(i, frame);
				i++;
			}
		}
		
		// Write 16 bytes of padding, as datalog reader doesn't read the last 16 bytes of the file, to not load
		// potentially truncated or corrupted data.
		for (int i = 0; i < 16; i++) {
			byteWriter.write(0);
		}
		
		LogReader reader = new LogReader(new DataLogReader(ByteBuffer.wrap(byteWriter.toByteArray())));
		
		int i = 0;
		for (LogFrame frame : frames) {
			LogFrame readFrame = reader.read();
			assertEquals(frame, readFrame, "Frame " + i + " does not match");
			i++;
		}
	}
}
