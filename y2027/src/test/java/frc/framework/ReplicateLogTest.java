package frc.framework;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.datalog.DataLogReader;
import frc.framework.logging.LogFrame;
import frc.framework.logging.LogReader;
import frc.framework.logging.LogWriter;
import org.junit.jupiter.api.Test;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;

public class ReplicateLogTest {
	@Test
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
