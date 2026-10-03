package frc.framework.logging;

import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Handles protobuf serialization &amp; deserialization for Rotation3ds
 */
public class Rotation3dStruct implements CustomStruct<Rotation3d, SystemLog.Rotation3D> {
	/**
	 * Singleton
	 */
	public static final Rotation3dStruct instance = new Rotation3dStruct();
	
	@Override
	public Rotation3d deserialize(SystemLog.Rotation3D rotation3D, LogReader reader) {
		return new Rotation3d(rotation3D.getRadiansX(), rotation3D.getRadiansY(), rotation3D.getRadiansZ());
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.ROTATION3D_FIELD_NUMBER;
	}
	
	@Override
	public Class<Rotation3d> getUnserializedClass() {
		return Rotation3d.class;
	}
	
	@Override
	public SystemLog.Value serialize(Rotation3d value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setRotation3D(
				SystemLog.Rotation3D.newBuilder()
					.setRadiansX(value.getX())
					.setRadiansY(value.getY())
					.setRadiansX(value.getZ())
			)
			.build();
	}
}
