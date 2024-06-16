package frc.robot.subsystems.drive.REVSwerve.SwerveModules;

public class REVSwerveModuleEncoderConstants {
	private double[] encoderConstantValues;

	/**
	 * Wrapper class designed to hold the encoder values associated with a REV
	 * Swerve Drive in the 135-Blocks framework
	 * 
	 * @param driveEncoderRot2Meter       The amount of rotations of the drive
	 *                                       motor that it takes for the module
	 *                                       to roll a meter
	 * @param driveEncoderRPM2MeterPerSec The amount of rotations per minute of
	 *                                       the drive motor that it takes for
	 *                                       the module to roll one meter per
	 *                                       second
	 * @param turningEncoderRot2Rad       The amount of rotations of the turning
	 *                                       motor that it takes for the module
	 *                                       to rotate one radian
	 * @param turningEncoderRPM2RadPerSec The amount of rotations per minute of
	 *                                       the turning motor that it takes for
	 *                                       the module to rotate one radian per
	 *                                       second
	 */
	public REVSwerveModuleEncoderConstants(double driveEncoderRot2Meter,
			double driveEncoderRPM2MeterPerSec, double turningEncoderRot2Rad,
			double turningEncoderRPM2RadPerSec) {
		encoderConstantValues = new double[] { driveEncoderRot2Meter,
				driveEncoderRPM2MeterPerSec, turningEncoderRot2Rad,
				turningEncoderRPM2RadPerSec
		};
	}

	public double getDriveEncoderRot2Meter() { return encoderConstantValues[0]; }

	public double getDriveEncoderRPM2MeterPerSec() {
		return encoderConstantValues[1];
	}

	public double getTurningEncoderRot2Rad() { return encoderConstantValues[2]; }

	public double getTurningEncoderRPM2RadPerSec() {
		return encoderConstantValues[3];
	}
}
