package frc.framework.commonrobot;

import frc.framework.logging.CustomStruct;
import frc.framework.logging.LogReader;
import frc.framework.logging.LogWriter;
import frc.framework.logging.SystemLog;

import java.util.Map;

public class ControllerStateStruct implements CustomStruct<ControllerState, SystemLog.ControllerState> {
	public static ControllerStateStruct instance = new ControllerStateStruct();
	
	@Override
	public ControllerState deserialize(SystemLog.ControllerState message, LogReader reader) {
		ControllerState result = new ControllerState();
		
		for (Map.Entry<Integer, Double> pair : message.getAxesMap().entrySet()) {
			result.setAxis(ControllerState.Axis.fromOrdinal(pair.getKey()), pair.getValue());
		}
		
		for (Map.Entry<Integer, Boolean> pair : message.getButtonsMap().entrySet()) {
			result.setButtonState(ControllerState.Button.fromOrdinal(pair.getKey()), pair.getValue());
		}
		
		return result;
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.CONTROLLERSTATE_FIELD_NUMBER;
	}
	
	@Override
	public Class<ControllerState> getUnserializedClass() {
		return ControllerState.class;
	}
	
	
	@Override
	public SystemLog.Value serialize(ControllerState value, LogWriter writer) {
		var builder = SystemLog.ControllerState.newBuilder();
		
		for (Map.Entry<ControllerState.Axis, Double> pair : value.getAllAxes()) {
			builder.putAxes(pair.getKey().ordinal(), pair.getValue());
		}
		for (Map.Entry<ControllerState.Button, Boolean> pair : value.getAllButtons()) {
			builder.putButtons(pair.getKey().ordinal(), pair.getValue());
		}
		
		return SystemLog.Value.newBuilder().setControllerState(builder).build();
	}
}
