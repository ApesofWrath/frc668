package frc.framework.logging;

import frc.framework.commonrobot.ControllerStateStruct;

import java.util.ArrayList;

public class Logging {
	public static ArrayList<CustomStruct<?, ?>> customStructs = new ArrayList<>();
	
	static {
		customStructs.add(ArrayStruct.instance);
		customStructs.add(IntegerStruct.instance);
		customStructs.add(LongStruct.instance);
		customStructs.add(DoubleStruct.instance);
		customStructs.add(FloatStruct.instance);
		customStructs.add(StringStruct.instance);
		customStructs.add(ControllerStateStruct.instance);
	}
}
