package frc.robot.subsystems.drive.REVSwerve.SwerveModules;

import frc.robot.subsystems.drive.REVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;

public interface REVSwerveModuleContainers {
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
			DriveConstants.kModuleTranslations[0]),
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
					DriveConstants.kModuleTranslations[1]),
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
					DriveConstants.kModuleTranslations[2]),
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
					DriveConstants.kModuleTranslations[3]);
}
