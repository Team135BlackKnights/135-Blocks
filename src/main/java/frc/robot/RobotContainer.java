// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.drive.SwerveC;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.CTREMecanum.CTREMecanumConstantContainer;
import frc.robot.subsystems.drive.CTREMecanum.CTREMecanumS;
import frc.robot.subsystems.drive.CTRESwerve.CTRESwerveS;
import frc.robot.subsystems.drive.CTRESwerve.Telemetry;
import frc.robot.subsystems.drive.CTRESwerve.TunerConstants;
import frc.robot.subsystems.drive.CTRETank.CTRETankConstantContainer;
import frc.robot.subsystems.drive.CTRETank.CTRETankS;
import frc.robot.subsystems.drive.REVMecanum.REVMecanumConstantContainer;
import frc.robot.subsystems.drive.REVMecanum.REVMecanumS;
import frc.robot.subsystems.drive.REVSwerve.REVSwerveS;
import frc.robot.subsystems.drive.REVSwerve.SwerveModules.REVSwerveModuleContainers;
import frc.robot.subsystems.drive.REVTank.REVTankConstantContainer;
import frc.robot.subsystems.drive.REVTank.REVTankS;
import frc.robot.utils.RunTest;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.subsystems.drive.REVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.revrobotics.CANSparkBase.IdleMode;
import java.util.List;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Arrays;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	private Telemetry logger = null;
	private final SendableChooser<Command> autoChooser;
	static PowerDistribution PDH = new PowerDistribution(
			Constants.PowerDistributionID, PowerDistribution.ModuleType.kRev);
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static XboxController testingController = new XboxController(5);
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			//yButtonDrive = new JoystickButton(driveController, 4), //used for DriveToPose
			aButtonTest = new JoystickButton(testingController, 1),
			bButtonTest = new JoystickButton(testingController, 2),
			xButtonTest = new JoystickButton(testingController, 3),
			yButtonTest = new JoystickButton(testingController, 4),
			leftBumperTest = new JoystickButton(testingController, 5),
			rightBumperTest = new JoystickButton(testingController, 6),
			selectButtonTest = new JoystickButton(testingController, 7),
			startButtonTest = new JoystickButton(testingController, 8);
	static int currentTest = 0;
 	public static Field2d field = new Field2d();
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
			switch (DriveConstants.robotMotorController) {
			case CTRE_MOTORS:
				drivetrainS = new CTRETankS(new CTRETankConstantContainer(30,
						DriveConstants.kFrontLeftDrivePort,
						DriveConstants.kBackLeftDrivePort,
						DriveConstants.kFrontRightDrivePort,
						DriveConstants.kBackRightDrivePort,
						TrainConstants.kDriveMotorGearRatio,
						DriveConstants.kChassisLength,
						TrainConstants.kDriveEncoderRot2Meter,
						Units.inchesToMeters(6), false, false,
						DriveConstants.kMaxSpeedMetersPerSecond));
				break;
			case NEO_SPARK_MAX:
			case VORTEX_SPARK_FLEX:
				//10, 11, 12, 13, false, false, false, false, IdleMode.kBrake, 80, 7.5, Units.inchesToMeters(6)
				drivetrainS = new REVTankS(new REVTankConstantContainer(10, 11, 12,
						13, false, false, false, false, IdleMode.kBrake, 80, 7.5,
						Units.inchesToMeters(6), DriveConstants.kChassisLength));
				break;
			}
			break;
		case MECANUM:
			switch (DriveConstants.robotMotorController) {
			case CTRE_MOTORS:
				drivetrainS = new CTREMecanumS(new CTREMecanumConstantContainer(30,
						DriveConstants.kFrontLeftDrivePort,
						DriveConstants.kBackLeftDrivePort,
						DriveConstants.kFrontRightDrivePort,
						DriveConstants.kBackRightDrivePort,
						DriveConstants.kChassisWidth,
						TrainConstants.kDriveMotorGearRatio,
						TrainConstants.kDriveEncoderRot2Meter,
						DriveConstants.kMaxSpeedMetersPerSecond,
						DriveConstants.kDriveBaseRadius,
						DriveConstants.kModuleTranslations));
				break;
			case NEO_SPARK_MAX:
			case VORTEX_SPARK_FLEX:
				//10, 11, 12, 13, 80, 7.5,
				drivetrainS = new REVMecanumS(new REVMecanumConstantContainer(10,
						11, 12, 13, 80, 7.5, TrainConstants.kWheelDiameter,
						DriveConstants.kModuleTranslations, Units.inchesToMeters(6)));
				break;
			}
			break;
		//Placeholder values
		default:
			throw new IllegalArgumentException(
					"Unknown implementation type, please check DriveConstants.java!");
		}
		drivetrainS.setDefaultCommand(new SwerveC(drivetrainS));
		List<Pair<String,Command> > autoCommands = Arrays.asList(
			//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
	   );
		Pathfinding.setPathfinder(new LocalADStarAK());
		NamedCommands.registerCommands(autoCommands);
		 if (Constants.isCompetition) {
      	PPLibTelemetry.enableCompetitionMode();
    	}
		PathfindingCommand.warmupCommand().schedule();
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.onChange(
        auto -> {
			try{
				field.getObject("path").setPoses(PathPlannerPath.fromChoreoTrajectory(auto.getName()).getPathPoses());
			}
			catch (Exception e){
				System.err.println("No found path for" + auto.getName());
				field.getObject("path").setPoses(new Pose2d[]{new Pose2d(-50,-50,new Rotation2d()),new Pose2d(-50.2,-50,new Rotation2d())});
			}
        });
		// Configure the trigger bindings
		configureBindings();
	}

	private void configureBindings() {
		xButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.whileTrue(PathFinder.goToPose(new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))), new PathConstraints(5.21, 10, 3, 6),drivetrainS)); //new InstantCommand(() -> drivetrainS.zeroHeading())
		yButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, true, drivetrainS));
		bButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, true, drivetrainS));
		aButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, false, drivetrainS));
		xButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, false, drivetrainS));
		//Example Drive To 2024 Amp Pose, Bind to what you need.
		//yButtonDrive.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest).negate()).whileTrue(new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90)))));
		//swerve DRIVE tests
		//When user hits right bumper, go to next test, or wrap back to starting test for SysID.
		rightBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == Constants.SysIdRoutines.values().length - 1) {
				currentTest = 0;
				System.out.println("looping");
			} else {
				currentTest++;
			}
		}));
		//When user hits left bumper, go to next test, or wrap back to starting test for SysID.
		leftBumperTest.onTrue(new InstantCommand(() -> {
			if (currentTest == 0) {
				currentTest = Constants.SysIdRoutines.values().length - 1;
				System.out.println("looping");
			} else {
				currentTest--;
			}
		}));
		//When using CTRE, be sure to hit Start so that the motors are logged via CTRE (For SysId)
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

	/**
	 * For SIMULATION ONLY, return the estimated current draw of the robot.
	 * 
	 * @return Current in amps.
	 */
	public static double[] getCurrentDraw() {
		return new double[] {0// Math.min(drivetrainS.getCurrent(), 200) //Enable when you need to test voltages. It can cause weird module behaviour, in which you restart
		};
	}
}