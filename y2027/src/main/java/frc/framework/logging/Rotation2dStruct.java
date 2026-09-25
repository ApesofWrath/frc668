package frc.framework.logging;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Handles protobuf serialization &amp; deserialization for Rotation2ds
 */
public class Rotation2dStruct implements CustomStruct<Rotation2d, SystemLog.Rotation2D> {
	/**
	 * Singleton
	 */
	public static final Rotation2dStruct instance = new Rotation2dStruct();
	
	@Override
	public Rotation2d deserialize(SystemLog.Rotation2D rotation2D, LogReader reader) {
		return new Rotation2d(rotation2D.getRadians());
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.ROTATION2D_FIELD_NUMBER;
	}
	
	@Override
	public Class<Rotation2d> getUnserializedClass() {
		return Rotation2d.class;
	}
	
	@Override
	public SystemLog.Value serialize(Rotation2d value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setRotation2D(SystemLog.Rotation2D.newBuilder().setRadians(value.getRadians()))
			.build();
	}
}
