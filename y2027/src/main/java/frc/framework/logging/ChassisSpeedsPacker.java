package frc.framework.logging;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsPacker implements Packer<ChassisSpeeds> {
	@Override
	public Class<ChassisSpeeds> getDataType() {
		return ChassisSpeeds.class;
	}
	
	@Override
	public ChassisSpeeds packFields(String key, PackHelper helper) {
		Double vx = helper.getValue(DataLogUtils.joinKey(key, "vx"), Double.class);
		Double vy = helper.getValue(DataLogUtils.joinKey(key, "vy"), Double.class);
		Double omega = helper.getValue(DataLogUtils.joinKey(key, "omega"), Double.class);
		
		return new ChassisSpeeds(vx, vy, omega);
	}
	
	@Override
	public void unpackFields(ChassisSpeeds value, String key, PackHelper packHelper) {
		packHelper.addUnpackedField(DataLogUtils.joinKey(key, "vx"), value.vxMetersPerSecond);
		packHelper.addUnpackedField(DataLogUtils.joinKey(key, "vy"), value.vyMetersPerSecond);
		packHelper.addUnpackedField(DataLogUtils.joinKey(key, "omega"), value.omegaRadiansPerSecond);
	}
}
