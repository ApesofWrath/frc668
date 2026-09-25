package frc.framework.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Handles protobuf serialization &amp; deserialization for Pose2ds
 */
public class Pose2dStruct implements CustomStruct<Pose2d, SystemLog.Pose2D> {
	/**
	 * Singleton
	 */
	public static final Pose2dStruct instance = new Pose2dStruct();
	
	@Override
	public Pose2d deserialize(SystemLog.Pose2D pose2D, LogReader reader) {
		return new Pose2d(pose2D.getX(), pose2D.getY(), Rotation2d.fromRadians(pose2D.getRotation()));
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.POSE2D_FIELD_NUMBER;
	}
	
	@Override
	public Class<Pose2d> getUnserializedClass() {
		return Pose2d.class;
	}
	
	@Override
	public SystemLog.Value serialize(Pose2d value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setPose2D(
				SystemLog.Pose2D.newBuilder()
					.setX(value.getX())
					.setY(value.getY())
					.setRotation(value.getRotation().getRadians())
			)
			.build();
	}
}
