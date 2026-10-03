package frc.framework.logging;

import frc.framework.OpMode;

/**
 * Handles protobuf serialization &amp; deserialization for OpModes
 */
public class OpModeStruct implements CustomStruct<OpMode, SystemLog.OpMode> {
	/**
	 * Singleton
	 */
	public static final OpModeStruct instance = new OpModeStruct();
	
	@Override
	public OpMode deserialize(SystemLog.OpMode opMode, LogReader reader) {
		return switch (opMode) {
		case Disabled, UNRECOGNIZED -> OpMode.Disabled;
		case Autonomous -> OpMode.Autonomous;
		case Teleop -> OpMode.Teleop;
		case Test -> OpMode.Test;
		};
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.TRANSLATION2D_FIELD_NUMBER;
	}
	
	@Override
	public Class<OpMode> getUnserializedClass() {
		return OpMode.class;
	}
	
	@Override
	public SystemLog.Value serialize(OpMode value, LogWriter writer) {
		return SystemLog.Value.newBuilder().setOpMode(switch (value) {
		case Disabled -> SystemLog.OpMode.Disabled;
		case Autonomous -> SystemLog.OpMode.Autonomous;
		case Teleop -> SystemLog.OpMode.Teleop;
		case Test -> SystemLog.OpMode.Test;
		}).build();
	}
}
