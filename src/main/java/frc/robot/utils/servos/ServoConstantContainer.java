package frc.robot.utils.servos;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Contains the PWM Port IDs 
 */
public class ServoConstantContainer {
	public static final int servoPWMPort = 1;

	public class ServoMotorConstants{
		//Free current amps is a random guess
		public static DCMotor SimREVSmartServo = new DCMotor(5, 1.32389775, 1, .4, 0.124666375, 1);
	}
	public enum SimServoMode {
		INRANGE, CONTINUOUS
	}

	public enum ServoType { REVSmartServo; }
}
