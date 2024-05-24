// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.drive.SwerveC;
import frc.robot.commands.drive.vision.DriveToAITarget;
import frc.robot.subsystems.drive.REVSwerve.SwerveModules.REVSwerveS;

import frc.robot.subsystems.vision.PhotonVisionS;
import frc.robot.utils.vision.VisionConstants;

import com.pathplanner.lib.auto.AutoBuilder;
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
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public final static REVSwerveS swerveS = new REVSwerveS();
	@SuppressWarnings("unused")
	private final PhotonVisionS photonVisionS = new PhotonVisionS();
	private final SendableChooser<Command> autoChooser;

	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	static JoystickButton 
	xButtonDrive = new JoystickButton(driveController, 3),
	selectButtonDrive = new JoystickButton(driveController, 7),
	startButtonDrive = new JoystickButton(driveController, 8);

	POVButton 
	povRightDrive = new POVButton(driveController, 90),
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
		swerveS.setDefaultCommand(new SwerveC(swerveS));
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData("Auto Chooser", autoChooser);
		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		VisionConstants.Controls.autoIntake.whileTrue(new DriveToAITarget(swerveS));
			xButtonDrive.and(isDriving()).onTrue(
				new InstantCommand(() -> swerveS.zeroHeading()));
		//swerve DRIVE tests
		startButtonDrive.and(povUpDrive).whileTrue(
				swerveS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(povRightDrive).whileTrue(
				swerveS.sysIdQuasistaticDrive(SysIdRoutine.Direction.kReverse));
		startButtonDrive.and(povDownDrive).whileTrue(
				swerveS.sysIdDynamicDrive(SysIdRoutine.Direction.kForward));
		startButtonDrive.and(povLeftDrive).whileTrue(
				swerveS.sysIdDynamicDrive(SysIdRoutine.Direction.kReverse));
		//swerve TURNING tests
		selectButtonDrive.and(povUpDrive).whileTrue(
				swerveS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kForward));
		selectButtonDrive.and(povRightDrive).whileTrue(
				swerveS.sysIdQuasistaticTurn(SysIdRoutine.Direction.kReverse));
		selectButtonDrive.and(povDownDrive).whileTrue(
				swerveS.sysIdDynamicTurn(SysIdRoutine.Direction.kForward));
		selectButtonDrive.and(povLeftDrive).whileTrue(
				swerveS.sysIdDynamicTurn(SysIdRoutine.Direction.kReverse));
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