package frc.framework.logging;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * Handles protobuf serialization &amp; deserialization for Pose3ds
 */
public class Pose3dStruct implements CustomStruct<Pose3d, SystemLog.Pose3D> {
	/**
	 * Singleton
	 */
	public static Pose3dStruct instance = new Pose3dStruct();
	
	@Override
	public Pose3d deserialize(SystemLog.Pose3D pose3D, LogReader reader) {
		return new Pose3d(
			pose3D.getX(),
			pose3D.getY(),
			pose3D.getZ(),
			new Rotation3d(pose3D.getRadiansX(), pose3D.getRadiansY(), pose3D.getRadiansZ())
		);
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.POSE2D_FIELD_NUMBER;
	}
	
	@Override
	public Class<Pose3d> getUnserializedClass() {
		return Pose3d.class;
	}
	
	@Override
	public SystemLog.Value serialize(Pose3d value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setPose3D(
				SystemLog.Pose3D.newBuilder()
					.setX(value.getX())
					.setY(value.getY())
					.setZ(value.getZ())
					.setRadiansX(value.getRotation().getX())
					.setRadiansY(value.getRotation().getY())
					.setRadiansY(value.getRotation().getZ())
			)
			.build();
	}
}
