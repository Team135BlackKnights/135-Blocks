package frc.robot.utils.drive;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;
import frc.robot.subsystems.drive.REVSwerve.REVModuleConstantContainer;

public class DriveConstants {
	//
	public static MotorVendor robotMotorController = MotorVendor.CTRE_MOTORS;
	public static driveTrainType driveType = driveTrainType.TANK;

	/**
	 * What motors and motorContollers are we using
	 */
	public enum MotorVendor {
		NEO_SPARK_MAX, VORTEX_SPARK_FLEX, CTRE_MOTORS
	}

	/**
	 * The drivetrain type
	 */
	public enum driveTrainType {
		SWERVE, TANK, MECANUM
	}

	public static boolean fieldOriented = true;
	//135-Blocks was tested on a chassis with all CANSparkMaxes, as well as all Kraken-x60s.
	public static final double kChassisWidth = Units.inchesToMeters(24.25), // Distance between Left and Right wheels
			kChassisLength = Units.inchesToMeters(24.25), // Distance betwwen Front and Back wheels
			kDriveBaseRadius = Units.inchesToMeters(Math.sqrt(
					kChassisLength * kChassisLength + kChassisWidth * kChassisWidth)
					/ 2),
			// Distance from center of robot to the farthest module
			kMaxSpeedMetersPerSecond = Units.feetToMeters(15.1), //15.1
			kMaxTurningSpeedRadPerSec = 3.914667 * 2 * Math.PI, // 1.33655 *2 *Math.PI
			kTeleDriveMaxAcceleration = Units.feetToMeters(12), // guess
			kTeleTurningMaxAcceleration = 2 * Math.PI, // guess
			// To find these set them to zero, then turn the robot on and manually set the
			// wheels straight.
			// The encoder values being read are then your new Offset values
			kFrontLeftAbsEncoderOffsetRad = 0.562867,
			kFrontRightAbsEncoderOffsetRad = 0.548137,
			kBackLeftAbsEncoderOffsetRad = 2 * Math.PI - 2.891372,
			kBackRightAbsEncoderOffsetRad = 2 * Math.PI - 0.116861;
	// kP = 0.1, kI = 0, kD = 0, kDistanceMultipler = .2; //for autoLock
	// Declare the position of each module
	public static final Translation2d[] kModuleTranslations = {
			new Translation2d(kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(kChassisLength / 2, -kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, kChassisWidth / 2),
			new Translation2d(-kChassisLength / 2, -kChassisWidth / 2)
	};
	public static int kFrontLeftDrivePort = 16, // 10
			kFrontLeftTurningPort = 17, // 20
			kFrontLeftAbsEncoderPort = 2, // 1
			kFrontRightDrivePort = 10, // 11
			kFrontRightTurningPort = 11, // 21
			kFrontRightAbsEncoderPort = 0, // 2
			kBackLeftDrivePort = 14, // 13
			kBackLeftTurningPort = 15, // 23
			kBackLeftAbsEncoderPort = 3, // 3
			kBackRightDrivePort = 12, // 14
			kBackRightTurningPort = 13, // 24
			kBackRightAbsEncoderPort = 1; // 4
	public static boolean kFrontLeftDriveReversed = true,
			kFrontLeftTurningReversed = true, kFrontLeftAbsEncoderReversed = false,
			kFrontRightDriveReversed = false, kFrontRightTurningReversed = true,
			kFrontRightAbsEncoderReversed = false, kBackLeftDriveReversed = false,
			kBackLeftTurningReversed = true, kBackLeftAbsEncoderReversed = false,
			kBackRightDriveReversed = false, kBackRightTurningReversed = true,
			kBackRightAbsEncoderReversed = false;

	public static class TrainConstants {
		/**
		 * Which swerve module it is (SWERVE EXCLUSIVE)
		 */
		public enum ModulePosition {
			FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT
		}

		public static double kWheelDiameter = Units.inchesToMeters(3.873),
				kDriveMotorGearRatio = 6.75, kTurningMotorGearRatio = 150 / 7,
				kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI
						* kWheelDiameter,
				kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60,
				kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI,
				kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60,
				kDeadband = 0.1, kAutoDeadband = 0.0000001, kOverallP = 2.36975,
				kOverallSVolts = -.180747, kOverallVVoltSecondsPerRotation = 2.8303,
				kOverallAVoltSecondsSquaredPerRotation = 1.4715;
		public static final MotorConstantContainer overallTurningMotorConstantContainer = new MotorConstantContainer(
				.2907325, .002131625, .000203095, 1.07062 * 4, 0.019508),
				overallDriveMotorConstantContainer = new MotorConstantContainer(
						-.180747, 2.8303, 1.4715, 2.36975, 0),
				frontRightDriveMotorConstantContainer = new MotorConstantContainer(
						.04248, 2.9041, 1.52, 2.4646, 0),
				frontLeftDriveMotorConstantContainer = new MotorConstantContainer(
						.22934, 2.8559, 1.7338, 2.5896, 0),
				backRightDriveMotorConstantContainer = new MotorConstantContainer(
						.070421, 2.8607, 1.1811, 2.0873, 0),
				backLeftDriveMotorConstantContainer = new MotorConstantContainer(
						.01842, 2.7005, 1.4511, 2.3375, 0),
				frontLeftTurningMotorConstantContainer = new MotorConstantContainer(
						.34809, .0021885, .00019056, 1.0181, 0),
				frontRightTurningMotorConstantContainer = new MotorConstantContainer(
						.28984, .0021057, .00018697, .99768, 0),
				backLeftTurningMotorConstantContainer = new MotorConstantContainer(
						.26615, .26615, .0021315, 1.0521, 0),
				backRightTurningMotorConstantContainer = new MotorConstantContainer(
						.25885, .0021008, .0002368, 1.2362, 0);
	}

	public static class DriveSimConstants {
		// must be changed to new game piece locations!
		public static Pose3d[] fieldNoteTranslations = new Pose3d[] {
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(228),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center up 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(294),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center up 2
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(96),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center down 1
				new Pose3d(Units.inchesToMeters(325.625), Units.inchesToMeters(30),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // center down 2
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // BLUE CENTER
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // BLUE CENTER + 1
				new Pose3d(Units.inchesToMeters(114), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // BUE TOP
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(162),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // RED CENTER
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(219),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // RED CENTER + 1
				new Pose3d(Units.inchesToMeters(534.5), Units.inchesToMeters(276),
						Units.inchesToMeters(1), new Rotation3d(0, 0, 0)), // RED TOP
		};
	}

	public static interface REVSwerveModuleContainers {
		public static REVModuleConstantContainer frontLeftConstantContainer = new REVModuleConstantContainer(
				DriveConstants.kFrontLeftDrivePort,
				DriveConstants.kFrontLeftTurningPort,
				DriveConstants.kFrontLeftDriveReversed,
				DriveConstants.kFrontLeftTurningReversed,
				DriveConstants.kFrontLeftAbsEncoderOffsetRad,
				DriveConstants.kFrontLeftAbsEncoderReversed,
				ModulePosition.FRONT_LEFT,
				DriveConstants.TrainConstants.frontLeftDriveMotorConstantContainer,
				DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
				kModuleTranslations[0]),
				frontRightConstantContainer = new REVModuleConstantContainer(
						DriveConstants.kFrontRightDrivePort,
						DriveConstants.kFrontRightTurningPort,
						DriveConstants.kFrontRightDriveReversed,
						DriveConstants.kFrontRightTurningReversed,
						DriveConstants.kFrontRightAbsEncoderOffsetRad,
						DriveConstants.kFrontRightAbsEncoderReversed,
						ModulePosition.FRONT_RIGHT,
						DriveConstants.TrainConstants.frontRightDriveMotorConstantContainer,
						DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
						kModuleTranslations[1]),
				backLeftConstantContainer = new REVModuleConstantContainer(
						DriveConstants.kBackLeftDrivePort,
						DriveConstants.kBackLeftTurningPort,
						DriveConstants.kBackLeftDriveReversed,
						DriveConstants.kBackLeftTurningReversed,
						DriveConstants.kBackLeftAbsEncoderOffsetRad,
						DriveConstants.kBackLeftAbsEncoderReversed,
						ModulePosition.BACK_LEFT,
						DriveConstants.TrainConstants.backLeftDriveMotorConstantContainer,
						DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
						kModuleTranslations[2]),
				backRightConstantContainer = new REVModuleConstantContainer(
						DriveConstants.kBackRightDrivePort,
						DriveConstants.kBackRightTurningPort,
						DriveConstants.kBackRightDriveReversed,
						DriveConstants.kBackRightTurningReversed,
						DriveConstants.kBackRightAbsEncoderOffsetRad,
						DriveConstants.kBackRightAbsEncoderReversed,
						ModulePosition.BACK_RIGHT,
						DriveConstants.TrainConstants.backRightDriveMotorConstantContainer,
						DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
						kModuleTranslations[3]);
	}
}
