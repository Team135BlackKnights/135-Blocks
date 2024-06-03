package frc.robot.subsystems.drive.REVMecanum;

import edu.wpi.first.math.geometry.Translation2d;

public class REVMecanumConstantContainer {
	private int[] mecanumConstantInts;
	private double[] mecanumConstantDoubles;
	private Translation2d[] mecanumWheelTranslations;

	/**
	 * Contains all constants needed for a REV mecanum drive
	 * 
	 * @param frontLeftID           front left motor ID
	 * @param frontRightID          front right motor ID
	 * @param backLeftID            back left motor ID
	 * @param backRightID           back right motor ID
	 * @param maxAmps               max amps that the motor should be able to get
	 * @param gearing               gearing of the drivetrain (>1 means
	 *                                 reduction)
	 * @param kWheelDiameterMeters  diameter of the wheel in meters
	 * @param mecanumTranslation2ds the translation 2ds from the center of the
	 *                                 robot to each wheel
	 */
	public REVMecanumConstantContainer(int frontLeftID, int frontRightID,
			int backLeftID, int backRightID, int maxAmps, double gearing,
			double kWheelDiameterMeters, Translation2d[] mecanumTranslation2ds, double driveBaseRadius) {
		mecanumConstantInts = new int[] { frontLeftID, frontRightID, backLeftID,
				backRightID, maxAmps
		};
		mecanumConstantDoubles = new double[] { gearing, kWheelDiameterMeters, driveBaseRadius
		};
		mecanumWheelTranslations = mecanumTranslation2ds;
	}

	public int getFrontLeftID() { return mecanumConstantInts[0]; }

	public int getFrontRightID() { return mecanumConstantInts[1]; }

	public int getBackLeftID() { return mecanumConstantInts[2]; }

	public int getBackRightID() { return mecanumConstantInts[3]; }

	public int getMaxAmps() { return mecanumConstantInts[4]; }

	public double getGearing() { return mecanumConstantDoubles[0]; }

	public double getWheelDiameters() { return mecanumConstantDoubles[1]; }

	public double getDriveBaseRadius(){return mecanumConstantDoubles[2];}
	public Translation2d[] getTranslation2ds() {
		return mecanumWheelTranslations;
	}
}
