package frc.framework.commonrobot;

import frc.framework.logging.DataLogUtils;
import frc.framework.logging.PackHelper;
import frc.framework.logging.Packer;

public class ControllerStatePacker implements Packer<ControllerState> {
	@Override
	public Class<ControllerState> getDataType() {
		return ControllerState.class;
	}
	
	@Override
	public ControllerState packFields(String key, PackHelper helper) {
		ControllerState result = new ControllerState();
		
		for (ControllerState.Axis axis : ControllerState.Axis.values()) {
			try {
				result.setAxis(axis, helper.getValue(DataLogUtils.joinKey(key, "axis", axis.name()), Double.class));
			} catch (Exception e) {
				// That's alright, just skip loading this axis
			}
		}
		for (ControllerState.Button button : ControllerState.Button.values()) {
			try {
				result.setButtonState(
					button,
					helper.getValue(DataLogUtils.joinKey(key, "button", button.name()), Boolean.class)
				);
			} catch (Exception e) {
				// That's alright, just skip loading this axis
			}
		}
		
		return result;
	}
	
	@Override
	public void unpackFields(ControllerState value, String key, PackHelper packHelper) {
		for (ControllerState.Axis axis : ControllerState.Axis.values()) {
			packHelper.addUnpackedField(DataLogUtils.joinKey(key, "axis", axis.name()), value.getAxis(axis));
		}
		for (ControllerState.Button button : ControllerState.Button.values()) {
			packHelper.addUnpackedField(
				DataLogUtils.joinKey(key, "button", button.name()),
				value.getButtonState(button)
			);
		}
	}
}
