// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.drive.SwerveC;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.CTRESwerve.CTRESwerveS;
import frc.robot.subsystems.drive.CTRESwerve.Telemetry;
import frc.robot.subsystems.drive.CTRESwerve.TunerConstants;
import frc.robot.subsystems.drive.REVSwerve.REVSwerveS;
import frc.robot.subsystems.drive.Tank.TankS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.commands.drive.vision.DriveToAITarget;

import frc.robot.subsystems.vision.PhotonVisionS;
import frc.robot.utils.vision.VisionConstants;

import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	private Telemetry logger = null;
	@SuppressWarnings("unused")
	private final PhotonVisionS photonVisionS = new PhotonVisionS();
	private final SendableChooser<Command> autoChooser;
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			selectButtonDrive = new JoystickButton(driveController, 7),
			startButtonDrive = new JoystickButton(driveController, 8);
	POVButton povRightDrive = new POVButton(driveController, 90),
			povDownDrive = new POVButton(driveController, 180),
			povLeftDrive = new POVButton(driveController, 270),
			povUpDrive = new POVButton(driveController, 0);

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		//We check to see what drivetrain type we have here, and create the correct drivetrain system based on that. 
		//If we get something wacky, throw an error
		switch (DriveConstants.vendor) {
		case REV_SWERVE:
		drivetrainS = new REVSwerveS();
			break;
		case CTRE_SWERVE:
			logger = new Telemetry(DriveConstants.kMaxSpeedMetersPerSecond);
			drivetrainS = new CTRESwerveS(TunerConstants.DrivetrainConstants, logger,
					TunerConstants.Modules);
			break;
		case TANK:
			drivetrainS = new TankS(10,11,12,13,false,false,false,false,IdleMode.kBrake,80,7.5,Units.inchesToMeters(6));
			break;
		default:
			throw new IllegalArgumentException(
					"Unknown implementation type, please check DriveConstants.java!");
		}
		drivetrainS.setDefaultCommand(new SwerveC(drivetrainS));
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		xButtonDrive.and(isDriving())
				.onTrue(new InstantCommand(() -> drivetrainS.zeroHeading()));
		VisionConstants.Controls.autoIntake.whileTrue(new DriveToAITarget(drivetrainS));
		//swerve DRIVE tests
		startButtonDrive.and(povUpDrive).whileTrue(
				drivetrainS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(povRightDrive).whileTrue(
				drivetrainS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
		startButtonDrive.and(povDownDrive).whileTrue(
				drivetrainS.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(povLeftDrive).whileTrue(
				drivetrainS.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
		//These tests can only be done for swerve, so we use a case system to bind them
		switch (DriveConstants.vendor) {
			case REV_SWERVE:
				selectButtonDrive.and(povUpDrive).whileTrue(
					drivetrainS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kForward));
				selectButtonDrive.and(povRightDrive).whileTrue(
					drivetrainS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kReverse));
				selectButtonDrive.and(povDownDrive).whileTrue(
					drivetrainS.sysIdDynamicTurn(SysIdRoutine.Direction.kForward));
				selectButtonDrive.and(povLeftDrive).whileTrue(
					drivetrainS.sysIdDynamicTurn(SysIdRoutine.Direction.kReverse));
				break;
			case CTRE_SWERVE:
				selectButtonDrive.and(povUpDrive).whileTrue(
					drivetrainS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kForward));
				selectButtonDrive.and(povRightDrive).whileTrue(
					drivetrainS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kReverse));
				selectButtonDrive.and(povDownDrive).whileTrue(
					drivetrainS.sysIdDynamicTurn(SysIdRoutine.Direction.kForward));
				selectButtonDrive.and(povLeftDrive).whileTrue(
					drivetrainS.sysIdDynamicTurn(SysIdRoutine.Direction.kReverse));
		
			default:
				break;
		}

	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	public static BooleanSupplier isDriving() {
		BooleanSupplier returnVal;
		if (startButtonDrive.getAsBoolean() || selectButtonDrive.getAsBoolean()) {
			returnVal = () -> false;
			return returnVal; //currently doing a test
		}
		returnVal = () -> true;
		return returnVal;
	}
}