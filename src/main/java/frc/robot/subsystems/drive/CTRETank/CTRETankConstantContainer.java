package frc.robot.subsystems.drive.CTRETank;

public class CTRETankConstantContainer {
	private int[] CTRETankInt;
	private double[] CTRETankDouble;
	private boolean[] CTRETankBooleans;

	/**
	 * Container to hold the constants needed to create a CTRE Tank Drive
	 * 
	 * @param pigeonID                The CAN ID of the pigeon
	 * @param leftLeaderID             The CAN ID of the front left motor
	 * @param rightLeaderID            The CAN ID of the front right motor
	 * @param leftFollowerID              The CAN ID of the back left motor
	 * @param rightFollowerID             The CAN ID of the back right motor
	 * @param kDriveMotorGearRatio    The gear ratio of the drive motors (> 1 is
	 *                                   a reduction)
	 * @param chassisLength           The length of the chassis
	 * @param driveEncoderRot2Meter   How many meters one full rotation of the
	 *                                   motor takes the robot
	 * @param wheelDiameter           Diameter of the wheel, in meters
	 * @param frontLeftReversed       Whether the front left motor is reversed
	 * @param frontRightReversed      Whether the front right motor is reversed
	 * @param maxSpeedMetersPerSecond The max speed the robot can go, in meters
	 *                                   per second
	 */
	public CTRETankConstantContainer(int pigeonID, int leftLeaderID,
			int rightLeaderID, int leftFollowerID, int rightFollowerID,
			double kDriveMotorGearRatio, double chassisLength,
			double driveEncoderRot2Meter, double wheelDiameter,
			boolean frontLeftReversed, boolean frontRightReversed,
			double maxSpeedMetersPerSecond) {
		CTRETankInt = new int[] { pigeonID, leftLeaderID, rightLeaderID, leftFollowerID,
				rightFollowerID
		};
		CTRETankDouble = new double[] { kDriveMotorGearRatio, chassisLength,
				driveEncoderRot2Meter, wheelDiameter, maxSpeedMetersPerSecond
		};
		CTRETankBooleans = new boolean[] { frontLeftReversed, frontRightReversed
		};
	}

	public int getPigeonID() { return CTRETankInt[0]; }

	public int getLeftLeaderID() { return CTRETankInt[1]; }

	public int getRightLeaderID() { return CTRETankInt[2]; }

	public int getLeftFollowerID() { return CTRETankInt[3]; }

	public int getRightFollowerID() { return CTRETankInt[4]; }

	public double getDriveGearRatio() { return CTRETankDouble[0]; }

	public double getChassisLength() { return CTRETankDouble[1]; }

	public double getDriveEncoderRot2Meter() { return CTRETankDouble[2]; }

	public double getWheelDiameter() { return CTRETankDouble[3]; }

	public double getMaxSpeedMetersPerSecond() { return CTRETankDouble[4]; }

	public boolean getLeftLeaderReversed() { return CTRETankBooleans[0]; }

	public boolean getRightLeaderReversed() { return CTRETankBooleans[1]; }
}
