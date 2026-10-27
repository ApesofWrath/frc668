package frc.framework;

import frc.framework.commonrobot.RobotInformation;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertThrows;

public class WaitForTest extends RobotTest {
	@Test
	@DisplayName(
		"Wait for a false condition"
	)
	public void waitForNeverResolves() {
		assertThrows(RuntimeException.class, () -> {
			waitFor(() -> false, Seconds.of(1));
		});
	}
	
	@Test
	@DisplayName(
		"Wait for an eventually true condition"
	)
	public void waitForResolves() {
		waitFor(() -> get(RobotInformation.TIMESTAMP_VALUE).orDefault(0L) > 500, Seconds.of(1));
	}
}
