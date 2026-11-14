package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class Constants{
	public class IntakeConstants {
		public static final int ROLLER_TOP_MOTOR_CANID = 41;
		public static final String ROLLER_TOP_MOTOR_CAN_BUS = "rio";
		public static final InvertedValue ROLLER_TOP_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
	
	
		public static final int ROLLER_BOTTOM_MOTOR_CANID = 42;
		public static final String ROLLER_BOTTOM_MOTOR_CAN_BUS = "rio";
		public static final InvertedValue ROLLER_BOTTOM_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
	
		public static final int DEPLOY_MOTOR_CAN_ID = 50;
		public static final String DEPLOY_MOTOR_CAN_BUS = "rio";
		public static final InvertedValue DEPLOY_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
	
		public static final int DEPLOY_ENCODER_CAN_ID = 61;
		public static final String DEPLOY_ENCODER_CAN_BUS = "rio";
		public static final SensorDirectionValue DEPLOY_ENCODER_DIRECTION = SensorDirectionValue.Clockwise_Positive;
	
		public static final double DEPLOY_SENSOR_TO_MECHANISM_RATIO = 3.3333333;
		public static final double DEPLOY_ROTOR_TO_SENSOR_RATIO = 25.0;
		public static final double K_S = 0.25;
		public static final double K_V = 0.12;
		public static final double K_A = 0.0;
		public static final double K_P = 0.2;
		public static final double K_I = 0.0;
		public static final double K_D = 0.0;
		public static final double ACTIVE_ROLLER_SPEED_RPS = 50.0;
		public static final double SENSOR_TO_MECHANISM_RATIO = 1.79;

		/*the deployed and undeployed states are based off of what phoenix tuner says the
		  motor positions are when the intake is folded in or out, these are rounded values and should be updated */
		public static final double INTAKE_DEPLOYED_POSITION = 0.3; 
		public static final double INTAKE_UNDEPLOYED_POSITION = 0.1; 
	
	}
	public class HopperConstants{
		/*Right hopper: 59
		Left hopper: 55
		Indexer front: 18
		Indexer back: 19 */

		public static int HOPPER_LEFT_MOTOR_CANID = 59;
		public static int HOPPER_RIGHT_MOTOR_CANID = 55;
		public static int INDEXER_FRONT_MOTOR_CANID = 18;
		public static int INDEXER_BACK_MOTOR_CANID = 19;

	}
}
