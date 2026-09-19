package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class Constants {
	public class IntakeConstants {
		public static final double ROLLER_TOP_MOTOR_CANID = 41;
		public static final String ROLLER_TOP_MOTOR_CAN_BUS = "rio";
		public static final InvertedValue ROLLER_TOP_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
		
		
		public static final double ROLLER_BOTTOM_MOTOR_CANID = 42;
		public static final String ROLLER_BOTTOM_MOTOR_CAN_BUS = "rio";
		public static final InvertedValue ROLLER_BOTTOM_MOTOR_INVERTED = InvertedValue.CounterClockwise_Positive;
		
		public static final double DEPLOY_MOTOR_CAN_ID = 50;
		public static final String DEPLOY_MOTOR_CAN_BUS = "rio";
		public static final InvertedValue DEPLOY_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
		
		public static final double DEPLOY_ENCODER_CAN_ID = 61;
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
		
		
	}
}