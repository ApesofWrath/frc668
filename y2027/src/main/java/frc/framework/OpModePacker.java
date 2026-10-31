package frc.framework;

import frc.framework.logging.DataLogUtils;
import frc.framework.logging.PackHelper;
import frc.framework.logging.Packer;

public class OpModePacker implements Packer<OpMode> {
	@Override
	public Class<OpMode> getDataType() {
		return OpMode.class;
	}
	
	@Override
	public OpMode packFields(String key, PackHelper helper) {
		String id = helper.getValue(DataLogUtils.joinKey(key, "opmode_value"), String.class);
		
		return OpMode.valueOf(id);
	}
	
	@Override
	public void unpackFields(OpMode value, String key, PackHelper packHelper) {
		packHelper.addUnpackedField(DataLogUtils.joinKey(key, "opmode_value"), value.toString());
	}
}
