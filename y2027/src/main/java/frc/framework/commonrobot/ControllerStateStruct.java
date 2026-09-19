package frc.framework.commonrobot;

import frc.framework.logging.CustomStruct;
import frc.framework.logging.LogReader;
import frc.framework.logging.LogWriter;

import java.nio.ByteBuffer;
import java.util.Map;

public class ControllerStateStruct implements CustomStruct<ControllerState> {
	public static ControllerStateStruct instance = new ControllerStateStruct();
	
	@Override
	public ControllerState deserialize(ByteBuffer bb, LogReader logReader) {
		ControllerState result = new ControllerState();
		
		int axesCount = bb.getInt();
		
		for (int i = 0; i < axesCount; i++) {
			result.setAxis(ControllerState.Axis.fromOrdinal(bb.getInt()), bb.getDouble());
		}
		
		int buttonsCount = bb.getInt();
		
		for (int i = 0; i < buttonsCount; i++) {
			result.setButtonState(ControllerState.Button.fromOrdinal(bb.getInt()), bb.get() != 0);
		}
		
		return result;
	}
	
	@Override
	public Class<ControllerState> getDataClass() {
		return ControllerState.class;
	}
	
	@Override
	public String getTypeId() {
		return "ControllerState";
	}
	
	@Override
	public void serialize(ControllerState value, LogWriter writer, ByteBuffer buffer) {
		buffer.putInt(value.getAllAxes().size());
		for (Map.Entry<ControllerState.Axis, Double> entry : value.getAllAxes()) {
			buffer.putInt(entry.getKey().ordinal());
			buffer.putDouble(entry.getValue());
		}
		buffer.putInt(value.getAllButtons().size());
		for (Map.Entry<ControllerState.Button, Boolean> entry : value.getAllButtons()) {
			buffer.putInt(entry.getKey().ordinal());
			buffer.put(entry.getValue() ? (byte) 1 : (byte) 0);
		}
	}
}
