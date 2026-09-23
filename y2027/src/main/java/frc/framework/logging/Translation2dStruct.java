package frc.framework.logging;

import edu.wpi.first.math.geometry.Translation2d;

public class Translation2dStruct implements CustomStruct<Translation2d, SystemLog.Translation2D> {
	public static final Translation2dStruct instance = new Translation2dStruct();
	
	@Override
	public Translation2d deserialize(SystemLog.Translation2D translation2D, LogReader reader) {
		return new Translation2d(translation2D.getX(), translation2D.getY());
	}
	
	@Override
	public int getFieldIndex() {
		return SystemLog.Value.TRANSLATION2D_FIELD_NUMBER;
	}
	
	@Override
	public Class<Translation2d> getUnserializedClass() {
		return Translation2d.class;
	}
	
	@Override
	public SystemLog.Value serialize(Translation2d value, LogWriter writer) {
		return SystemLog.Value.newBuilder()
			.setTranslation2D(SystemLog.Translation2D.newBuilder().setX(value.getX()).setY(value.getY()))
			.build();
	}
}
