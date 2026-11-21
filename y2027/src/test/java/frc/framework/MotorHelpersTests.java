package frc.framework;

import frc.framework.commonrobot.MotorControl;
import frc.framework.commonrobot.MotorControlType;
import frc.framework.commonrobot.Motors;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.*;

public class MotorHelpersTests {
	@Test
	@DisplayName(
		"Motor control helpers work as intended"
	)
	public void motorControlHelpers() {
		Assertions.assertEquals(new MotorControl(MotorControlType.Position, 1), MotorControl.position(Radians.of(1)));
		Assertions.assertEquals(new MotorControl(MotorControlType.Position, 1), MotorControl.position(1));
		Assertions.assertEquals(
			new MotorControl(MotorControlType.Velocity, 1),
			MotorControl.velocity(RadiansPerSecond.of(1))
		);
		Assertions.assertEquals(new MotorControl(MotorControlType.Velocity, 1), MotorControl.velocity(1));
		Assertions.assertEquals(new MotorControl(MotorControlType.Voltage, 1), MotorControl.voltage(Volts.of(1)));
		Assertions.assertEquals(new MotorControl(MotorControlType.Voltage, 1), MotorControl.voltage(1));
	}
	
	@Test
	@DisplayName(
		"Value identifiers match between invocations"
	)
	public void motorValueIdentifiersMatch() {
		Assertions.assertEquals(
			Motors.controlIdentifier(TestMotorIds.Alpha),
			Motors.controlIdentifier(TestMotorIds.Alpha)
		);
		Assertions.assertEquals(
			Motors.controlIdentifier(TestMotorIds.Beta),
			Motors.controlIdentifier(TestMotorIds.Beta)
		);
		Assertions.assertEquals(
			Motors.controlIdentifier(TestMotorIds.Gamma),
			Motors.controlIdentifier(TestMotorIds.Gamma)
		);
	}
	
	@Test
	@DisplayName(
		"Value identifiers are not interchanged"
	)
	public void motorValueIdentifiersNotInterchanged() {
		Assertions.assertNotEquals(
			Motors.controlIdentifier(TestMotorIds.Alpha),
			Motors.controlIdentifier(TestMotorIds.Beta)
		);
		Assertions.assertNotEquals(
			Motors.controlIdentifier(TestMotorIds.Beta),
			Motors.controlIdentifier(TestMotorIds.Gamma)
		);
		Assertions.assertNotEquals(
			Motors.controlIdentifier(TestMotorIds.Gamma),
			Motors.controlIdentifier(TestMotorIds.Alpha)
		);
	}
}

enum TestMotorIds {
	Alpha,
	Beta,
	Gamma
}
