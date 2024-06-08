package frc.robot.subsystems.servos;

import frc.robot.utils.servos.ServoConstantContainer;
import frc.robot.utils.servos.ServoPackage;
import frc.robot.utils.servos.ServoConstantContainer.ServoNames;
import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoS extends SubsystemBase {
	private ServoPackage[] servoPackages = {
			new ServoPackage(ServoConstantContainer.leftServoPWMPort,
					SimServoMode.INRANGE, ServoType.REVSmartServo, 0, .02,
					ServoConstantContainer.leftLowerServoBound,
					ServoConstantContainer.leftUpperServoBound),
			new ServoPackage(ServoConstantContainer.rightServoPWMPort,
					SimServoMode.INRANGE, ServoType.REVSmartServo, -45, .02,
					ServoConstantContainer.rightLowerServoBound,
					ServoConstantContainer.rightUpperServoBound)
	};

	/**
	 * Set the servo to a specified angle
	 * 
	 * @param degrees the desired angle (in degrees)
	 */
	public void setServoDegrees(double degrees, ServoNames servoName) {
		switch (servoName) {
		case leftServo:
			servoPackages[0].setServoDegrees(degrees);
			break;
		case rightServo:
			servoPackages[1].setServoDegrees(degrees);
			break;
		}
	}

	/**
	 * Sets the servo to a certain percent. If it's continuous, sets a POSITION
	 * from 0 to 1, where 0 is maximum left and 1 is maximum right If it's in
	 * range, set it to a constant PERCENTAGE of the max velocity
	 * 
	 * @param percent the percent to set the servo to (-1 to 1) if the servo is
	 *                   continuous, the position to be set to from (0 to 1.0) if
	 *                   it is in range mode
	 */
	public void setServoPercent(double percent, ServoNames servoName) {
		switch (servoName) {
		case leftServo:
			servoPackages[0].setServoPercent(percent);
			break;
		case rightServo:
			servoPackages[1].setServoPercent(percent);
			break;
		}
	}

	/**
	 * @return the angular position of the servo in degrees
	 */
	public double getServoDegrees(ServoNames servoName) {
		switch (servoName) {
		case leftServo:
			return servoPackages[0].getServoDegrees();
		case rightServo:
			return servoPackages[1].getServoDegrees();
		default:
			return 0;
		}
	}

	@Override
	public void periodic() {
		for (ServoPackage servo : servoPackages) {
			servo.updateServoSim();
		}
	}
}
