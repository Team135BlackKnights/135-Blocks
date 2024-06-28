package frc.robot.subsystems.drive.OldREVSwerve.SwerveModules;

import frc.robot.subsystems.drive.OldREVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants.*;

public interface REVSwerveModuleContainers {
	public static REVModuleConstantContainer frontLeftConstantContainer = new REVModuleConstantContainer(
			DriveConstants.kFrontLeftDrivePort,
			DriveConstants.kFrontLeftTurningPort,
			DriveConstants.kFrontLeftDriveReversed,
			DriveConstants.kFrontLeftTurningReversed,
			DriveConstants.kFrontLeftAbsEncoderOffsetRad,
			DriveConstants.kMaxSpeedMetersPerSecond,
			TrainConstants.kDriveMotorGearRatio,
			TrainConstants.kTurningMotorGearRatio,
			DriveConstants.kFrontLeftAbsEncoderReversed, ModulePosition.FRONT_LEFT,
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer,
			DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
			DriveConstants.kModuleTranslations[0]),
			frontRightConstantContainer = new REVModuleConstantContainer(
					DriveConstants.kFrontRightDrivePort,
					DriveConstants.kFrontRightTurningPort,
					DriveConstants.kFrontRightDriveReversed,
					DriveConstants.kFrontRightTurningReversed,
					DriveConstants.kFrontRightAbsEncoderOffsetRad,
					DriveConstants.kMaxSpeedMetersPerSecond,
					TrainConstants.kDriveMotorGearRatio,
					TrainConstants.kTurningMotorGearRatio,
					DriveConstants.kFrontRightAbsEncoderReversed,
					ModulePosition.FRONT_RIGHT,
					DriveConstants.TrainConstants.overallDriveMotorConstantContainer,
					DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
					DriveConstants.kModuleTranslations[1]),
			backLeftConstantContainer = new REVModuleConstantContainer(
					DriveConstants.kBackLeftDrivePort,
					DriveConstants.kBackLeftTurningPort,
					DriveConstants.kBackLeftDriveReversed,
					DriveConstants.kBackLeftTurningReversed,
					DriveConstants.kBackLeftAbsEncoderOffsetRad,
					DriveConstants.kMaxSpeedMetersPerSecond,
								TrainConstants.kDriveMotorGearRatio,
			TrainConstants.kTurningMotorGearRatio,
					DriveConstants.kBackLeftAbsEncoderReversed,
					ModulePosition.BACK_LEFT,
					DriveConstants.TrainConstants.overallDriveMotorConstantContainer,
					DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
					DriveConstants.kModuleTranslations[2]),
			backRightConstantContainer = new REVModuleConstantContainer(
					DriveConstants.kBackRightDrivePort,
					DriveConstants.kBackRightTurningPort,
					DriveConstants.kBackRightDriveReversed,
					DriveConstants.kBackRightTurningReversed,
					DriveConstants.kBackRightAbsEncoderOffsetRad,
					DriveConstants.kMaxSpeedMetersPerSecond,
								TrainConstants.kDriveMotorGearRatio,
			TrainConstants.kTurningMotorGearRatio,
					DriveConstants.kBackRightAbsEncoderReversed,
					ModulePosition.BACK_RIGHT,
					DriveConstants.TrainConstants.overallDriveMotorConstantContainer,
					DriveConstants.TrainConstants.overallTurningMotorConstantContainer,
					DriveConstants.kModuleTranslations[3]);
}
