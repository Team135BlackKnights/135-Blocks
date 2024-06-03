package frc.robot.subsystems.drive.CTREMecanum;

import edu.wpi.first.math.geometry.Translation2d;

public class CTREMecanumConstantContainer {
	Translation2d[] mecanumTranslation2ds;
	int[] mecanumConstantInts;
	double[] mecanumConstantDoubles;

	/**
	 * Container that holds the constants to create a CTRE Mecanum drive
	 * 
	 * @param pigeonID                the CAN ID of the pigeon
	 * @param frontLeftID             the CAN ID of the front left motor
	 * @param backLeftID            the CAN ID of the back left motor
	 * @param frontRightID              the CAN ID of the front right motor
	 * @param backRightID             the CAN ID of the back right mtoro
	 * @param driveMotorGearRatio     the gear ratio of the drivetrain (>1 means
	 *                                   a reduction)
	 * @param driveEncoderRot2Meter   How many meters one full rotation of the
	 *                                   motor takes the robot
	 * @param maxSpeedMetersPerSecond the max speed of the robot in meters per
	 *                                   second
	 * @param driveBaseRadius         the radius of the drive base in meters
	 * @param moduleTranslations      an array of translation2ds that represent
	 *                                   each wheel's distance from the robot
	 *                                   center in the order {frontLeft,
	 *                                   frontRight, backLeft, backRight}
	 */
	public CTREMecanumConstantContainer(int pigeonID, int frontLeftID,
			int backLeftID, int frontRightID, int backRightID,
			double wheelDiameterMeters, double driveMotorGearRatio,
			double driveEncoderRot2Meter, double maxSpeedMetersPerSecond,
			double driveBaseRadius, Translation2d[] moduleTranslations) {
		mecanumTranslation2ds = moduleTranslations;
		mecanumConstantDoubles = new double[] { driveEncoderRot2Meter,
				maxSpeedMetersPerSecond, driveBaseRadius, wheelDiameterMeters, driveMotorGearRatio
		};
		mecanumConstantInts = new int[] { pigeonID, frontLeftID, frontRightID,
				backLeftID, backRightID
		};
	}

	public int getPigeonID() { return mecanumConstantInts[0]; }

	public int getFrontLeftID() { return mecanumConstantInts[1]; }

	public int getFrontRightID() { return mecanumConstantInts[2]; }

	public int getBackLeftID() { return mecanumConstantInts[3]; }

	public int getBackRightID() { return mecanumConstantInts[4]; }

	public double getDriveEncoderRot2Meter() { return mecanumConstantDoubles[0]; }

	public double getMaxSpeedMetersPerSecond() {
		return mecanumConstantDoubles[1];
	}

	public double getDriveBaseRadius() { return mecanumConstantDoubles[2]; }

	public double getWheelDiameterMeters() { return mecanumConstantDoubles[3]; }
	
	public double getDriveMotorGearRatio(){
		return mecanumConstantDoubles[4];
	}

	public Translation2d[] getTranslation2ds() { return mecanumTranslation2ds; }
}
