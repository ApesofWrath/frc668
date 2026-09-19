package frc.framework.logging;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.util.struct.Struct;
import frc.framework.commonrobot.ControllerStateStruct;

import java.util.ArrayList;

public class Logging {
	public static ArrayList<CustomStruct<?>> customStructs = new ArrayList<>();
	
	static {
		Struct<?>[] structs = new Struct[]{Translation2d.struct, Translation3d.struct, Pose3d.struct, Pose2d.struct, Rotation2d.struct, Rotation3d.struct};
		customStructs.add(ArrayStruct.instance);
		customStructs.add(IntegerStruct.instance);
		customStructs.add(LongStruct.instance);
		customStructs.add(DoubleStruct.instance);
		customStructs.add(FloatStruct.instance);
		customStructs.add(StringStruct.instance);
		customStructs.add(ControllerStateStruct.instance);
		
		for (Struct<?> struct : structs) {
			customStructs.add(StructUtils.toCustomStruct(struct));
		}
	}
}
