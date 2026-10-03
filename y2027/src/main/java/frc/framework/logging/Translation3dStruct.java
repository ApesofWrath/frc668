package frc.framework.logging;

import edu.wpi.first.math.geometry.Translation3d;

/**
 * Handles protobuf serialization &amp; deserialization for Translation3ds
 */
public class Translation3dStruct implements CustomStruct<Translation3d, SystemLog.Translation3D> {
	/**
	 * Singleton
	 */
	public static final Translation3dStruct instance = new Translation3dStruct();
	
	@Override
	public Translation3d deserialize(SystemLog.Translation3D translation3D, LogReader reader) {
		return new Translation3d(translation3D.getX(), translation3D.getY(), translation3D.getZ());
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.TRANSLATION2D_FIELD_NUMBER;
	}
	
	@Override
	public Class<Translation3d> getUnserializedClass() {
		return Translation3d.class;
	}
	
	@Override
	public SystemLog.Value serialize(Translation3d value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setTranslation3D(
				SystemLog.Translation3D.newBuilder().setX(value.getX()).setY(value.getY()).setZ(value.getZ())
			)
			.build();
	}
}
