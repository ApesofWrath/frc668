package frc.framework.logging;

import frc.framework.commonrobot.ControllerStateStruct;

import java.util.ArrayList;

public class Logging {
	public static ArrayList<CustomStruct<?, ?>> customStructs = new ArrayList<>();
	
	static {
		// primitives
		customStructs.add(ArrayStruct.instance);
		customStructs.add(IntegerStruct.instance);
		customStructs.add(LongStruct.instance);
		customStructs.add(DoubleStruct.instance);
		customStructs.add(FloatStruct.instance);
		customStructs.add(StringStruct.instance);
		
		// custom
		customStructs.add(ControllerStateStruct.instance);
		
		// wpilib geometry
		customStructs.add(Pose2dStruct.instance);
		customStructs.add(Pose3dStruct.instance);
		customStructs.add(Translation2dStruct.instance);
		customStructs.add(Translation3dStruct.instance);
		customStructs.add(Rotation2dStruct.instance);
		customStructs.add(Rotation3dStruct.instance);
		customStructs.add(QuaternionStruct.instance);
	}
}
