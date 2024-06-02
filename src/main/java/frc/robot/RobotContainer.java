// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.drive.SwerveC;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.CTREMecanum.CTREMecanumS;
import frc.robot.subsystems.drive.CTRESwerve.CTRESwerveS;
import frc.robot.subsystems.drive.CTRESwerve.Telemetry;
import frc.robot.subsystems.drive.CTRESwerve.TunerConstants;
import frc.robot.subsystems.drive.REVMecanum.REVMecanumS;
import frc.robot.subsystems.drive.REVSwerve.REVSwerveS;

import frc.robot.subsystems.drive.REVTank.REVTankS;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.subsystems.drive.REVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants.REVSwerveModuleContainers;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	private Telemetry logger = null;
	private final SendableChooser<Command> autoChooser;
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static XboxController testingController = new XboxController(5);
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			aButtonTest = new JoystickButton(testingController, 1),
			bButtonTest = new JoystickButton(testingController, 2),
			xButtonTest = new JoystickButton(testingController, 3),
			yButtonTest = new JoystickButton(testingController, 4),
			leftBumperTest = new JoystickButton(testingController, 5),
			rightBumperTest = new JoystickButton(testingController, 6),
			selectButtonTest = new JoystickButton(testingController, 7),
			startButtonTest = new JoystickButton(testingController, 8);
	static int currentTest = 0;

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
	 */
	public RobotContainer() {
		//We check to see what drivetrain type we have here, and create the correct drivetrain system based on that. 
		//If we get something wacky, throw an error
		switch (DriveConstants.driveType) {
		case SWERVE:
			switch (DriveConstants.robotMotorController) {
			case CTRE_MOTORS:
				logger = new Telemetry(DriveConstants.kMaxSpeedMetersPerSecond);
				drivetrainS = new CTRESwerveS(TunerConstants.DrivetrainConstants,
						logger, TunerConstants.Modules);
				break;
			case NEO_SPARK_MAX:
			case VORTEX_SPARK_FLEX:
				drivetrainS = new REVSwerveS(new REVModuleConstantContainer[] {
						REVSwerveModuleContainers.frontLeftConstantContainer,
						REVSwerveModuleContainers.frontRightConstantContainer,
						REVSwerveModuleContainers.backLeftConstantContainer,
						REVSwerveModuleContainers.backRightConstantContainer
				}, DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.kDriveBaseRadius);
				break;
			}
			break;
		case TANK:
			drivetrainS = new REVTankS(10, 11, 12, 13, false, false, false, false,
					IdleMode.kBrake, 80, 7.5, Units.inchesToMeters(6));
			break;
		case MECANUM:
			switch (DriveConstants.robotMotorController) {
			case CTRE_MOTORS:
				drivetrainS = new CTREMecanumS();
				break;
			case NEO_SPARK_MAX:
			case VORTEX_SPARK_FLEX:
				drivetrainS = new REVMecanumS(10, 11, 12, 13, 80, 7.5,
						Units.inchesToMeters(6));
				break;
			}
			break;
		//Placeholder values
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
		//swerve DRIVE tests
		rightBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == Constants.SysIdRoutines.values().length - 1) {
				currentTest = 0;
			} else {
				currentTest++;
			}
		}));
		leftBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == 0) {
				currentTest = Constants.SysIdRoutines.values().length - 1;
			} else {
				currentTest--;
			}
		}));
		selectButtonTest.onTrue(Commands.runOnce(SignalLogger::stop));
		startButtonTest.onTrue(Commands.runOnce(SignalLogger::start));
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
		if (aButtonTest.getAsBoolean() || bButtonTest.getAsBoolean()
				|| xButtonTest.getAsBoolean() || yButtonTest.getAsBoolean()) {
			returnVal = () -> false;
			return returnVal; //currently doing a test
		}
		returnVal = () -> true;
		return returnVal;
	}
}