package frc.framework.logging;

import edu.wpi.first.math.geometry.Quaternion;

/**
 * Handles protobuf serialization &amp; deserialization for quaternions
 */
public class QuaternionStruct implements CustomStruct<Quaternion, SystemLog.Quaternion> {
	/**
	 * Singleton
	 */
	public static final QuaternionStruct instance = new QuaternionStruct();
	
	@Override
	public Quaternion deserialize(SystemLog.Quaternion quaternion, LogReader reader) {
		return new Quaternion(quaternion.getW(), quaternion.getX(), quaternion.getY(), quaternion.getZ());
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.QUATERNION_FIELD_NUMBER;
	}
	
	@Override
	public Class<Quaternion> getUnserializedClass() {
		return Quaternion.class;
	}
	
	@Override
	public SystemLog.Value serialize(Quaternion value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setQuaternion(
				SystemLog.Quaternion.newBuilder()
					.setW(value.getW())
					.setX(value.getX())
					.setY(value.getY())
					.setZ(value.getZ())
			)
			.build();
	}
}
