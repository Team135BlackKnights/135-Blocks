package frc.robot.subsystems.drive.REVSwerve.SwerveModules;

import frc.robot.subsystems.drive.REVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants.*;

public interface REVSwerveModuleContainers {
	public static REVSwerveModuleEncoderConstants drivetrainEncoderConstants = new REVSwerveModuleEncoderConstants(TrainConstants.kDriveEncoderRot2Meter, TrainConstants.kDriveEncoderRPM2MeterPerSec, TrainConstants.kTurningEncoderRot2Rad, TrainConstants.kTurningEncoderRPM2RadPerSec);
	public static REVModuleConstantContainer frontLeftConstantContainer = new REVModuleConstantContainer(
			DriveConstants.kFrontLeftDrivePort,
			DriveConstants.kFrontLeftTurningPort,
			DriveConstants.kFrontLeftDriveReversed,
			DriveConstants.kFrontLeftTurningReversed,
			DriveConstants.kFrontLeftAbsEncoderOffsetRad,
			DriveConstants.kMaxSpeedMetersPerSecond,
			DriveConstants.kFrontLeftAbsEncoderReversed,
			ModulePosition.FRONT_LEFT,
			DriveConstants.TrainConstants.frontLeftDriveMotorConstantContainer,
			DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
			DriveConstants.kModuleTranslations[0],drivetrainEncoderConstants),
			frontRightConstantContainer = new REVModuleConstantContainer(
					DriveConstants.kFrontRightDrivePort,
					DriveConstants.kFrontRightTurningPort,
					DriveConstants.kFrontRightDriveReversed,
					DriveConstants.kFrontRightTurningReversed,
					DriveConstants.kFrontRightAbsEncoderOffsetRad,
					DriveConstants.kMaxSpeedMetersPerSecond,
					DriveConstants.kFrontRightAbsEncoderReversed,
					ModulePosition.FRONT_RIGHT,
					DriveConstants.TrainConstants.frontRightDriveMotorConstantContainer,
					DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
					DriveConstants.kModuleTranslations[1],drivetrainEncoderConstants),
			backLeftConstantContainer = new REVModuleConstantContainer(
					DriveConstants.kBackLeftDrivePort,
					DriveConstants.kBackLeftTurningPort,
					DriveConstants.kBackLeftDriveReversed,
					DriveConstants.kBackLeftTurningReversed,
					DriveConstants.kBackLeftAbsEncoderOffsetRad,
					DriveConstants.kMaxSpeedMetersPerSecond,
					DriveConstants.kBackLeftAbsEncoderReversed,
					ModulePosition.BACK_LEFT,
					DriveConstants.TrainConstants.backLeftDriveMotorConstantContainer,
					DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
					DriveConstants.kModuleTranslations[2],drivetrainEncoderConstants),
			backRightConstantContainer = new REVModuleConstantContainer(
					DriveConstants.kBackRightDrivePort,
					DriveConstants.kBackRightTurningPort,
					DriveConstants.kBackRightDriveReversed,
					DriveConstants.kBackRightTurningReversed,
					DriveConstants.kBackRightAbsEncoderOffsetRad,
					DriveConstants.kMaxSpeedMetersPerSecond,
					DriveConstants.kBackRightAbsEncoderReversed,
					ModulePosition.BACK_RIGHT,
					DriveConstants.TrainConstants.backRightDriveMotorConstantContainer,
					DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
					DriveConstants.kModuleTranslations[3],drivetrainEncoderConstants);
}
