package frc.framework.logging;

import com.google.protobuf.Descriptors;
import frc.framework.OpMode;

/**
 * Handles protobuf serialization &amp; deserialization for OpModes
 */
public class OpModeStruct implements CustomStruct<OpMode, Descriptors.EnumValueDescriptor> {
	/**
	 * Singleton
	 */
	public static final OpModeStruct instance = new OpModeStruct();
	
	@Override
	public OpMode deserialize(Descriptors.EnumValueDescriptor opMode, LogReader reader) {
		SystemLog.OpMode realOpmode = SystemLog.OpMode.valueOf(opMode);
		
		return switch (realOpmode) {
		case Disabled, UNRECOGNIZED -> OpMode.Disabled;
		case Autonomous -> OpMode.Autonomous;
		case Teleop -> OpMode.Teleop;
		case Test -> OpMode.Test;
		};
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.OPMODE_FIELD_NUMBER;
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
