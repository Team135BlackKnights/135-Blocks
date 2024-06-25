// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import frc.robot.commands.drive.DrivetrainC;
import frc.robot.commands.state_space.CTREDoubleJointedArmC;
import frc.robot.commands.state_space.CTREElevatorC;
import frc.robot.commands.state_space.SingleJointedArmC;
import frc.robot.commands.state_space.FlywheelC;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.CTREMecanum.CTREMecanumConstantContainer;
import frc.robot.subsystems.drive.CTREMecanum.CTREMecanumS;
import frc.robot.subsystems.drive.FastSwerve.Swerve;
import frc.robot.subsystems.drive.FastSwerve.GyroIO;
import frc.robot.subsystems.drive.FastSwerve.GyroIOPigeon2;
import frc.robot.subsystems.drive.FastSwerve.ModuleIO;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSim;
import frc.robot.subsystems.drive.FastSwerve.ModuleIOSparkMax;
import frc.robot.subsystems.drive.REVMecanum.REVMecanumConstantContainer;
import frc.robot.subsystems.drive.REVMecanum.REVMecanumS;
import frc.robot.subsystems.drive.Tank.DriveIO;
import frc.robot.subsystems.drive.Tank.DriveIOSim;
import frc.robot.subsystems.drive.Tank.DriveIOSparkMax;
import frc.robot.subsystems.drive.Tank.DriveIOTalonFX;
import frc.robot.subsystems.drive.Tank.Tank;
import frc.robot.subsystems.state_space.CTREDoubleJointedArmS;
import frc.robot.subsystems.state_space.CTREElevatorS;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIO;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOSim;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOSpark;
import frc.robot.subsystems.state_space.Flywheel.FlywheelIOTalon;
import frc.robot.subsystems.state_space.Flywheel.FlywheelS;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmIOSim;
import frc.robot.subsystems.state_space.SingleJointedArm.SingleJointedArmS;
import frc.robot.utils.RunTest;
import frc.robot.utils.drive.DriveConstants;

import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.state_space.StateSpaceConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.drive.PathFinder;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PPLibTelemetry;
import java.util.List;
import java.util.Optional;



import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * THIS CODE REQUIRES WPILIB 2024 AND PATHPLANNER 2024 IT WILL NOT WORK
 * OTHERWISE
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	public static DrivetrainS drivetrainS;
	public static FlywheelS flywheelS;
	public static SingleJointedArmS armS;
	public static final CTREElevatorS elevatorS = new CTREElevatorS();
	public static final CTREDoubleJointedArmS doubleJointedArmS = new CTREDoubleJointedArmS();
	private final SendableChooser<Command> autoChooser;
	static PowerDistribution PDH = new PowerDistribution(
			Constants.PowerDistributionID, PowerDistribution.ModuleType.kRev);
	public static XboxController driveController = new XboxController(0);
	public static XboxController manipController = new XboxController(1);
	public static XboxController testingController = new XboxController(5);
	public static Optional<Rotation2d> angleOverrider = Optional.empty();
	public static double angularSpeed = 0;
	static JoystickButton xButtonDrive = new JoystickButton(driveController, 3),
			//yButtonDrive = new JoystickButton(driveController, 4), //used for Aim/Drive to pose
			aButtonTest = new JoystickButton(testingController, 1),
			bButtonTest = new JoystickButton(testingController, 2),
			xButtonTest = new JoystickButton(testingController, 3),
			yButtonTest = new JoystickButton(testingController, 4),
			leftBumperTest = new JoystickButton(testingController, 5),
			rightBumperTest = new JoystickButton(testingController, 6),
			selectButtonTest = new JoystickButton(testingController, 7),
			startButtonTest = new JoystickButton(testingController, 8);
	public static int currentTest = 0, currentGamePieceStatus = 0;
	public static String currentPath = "";
	public static Field2d field = new Field2d();
   public static Pose2d opposingBotPose;

	// POVButton manipPOVZero = new POVButton(manipController, 0);
	// POVButton manipPOV180 = new POVButton(manipController, 180);
	/**
	 * The container for the robot. Contains subsystems, OI devices, and
	 * commands.
y	 * @throws NotActiveException IF mecanum and Replay
	 */
	public RobotContainer() {
		//We check to see what drivetrain type we have here, and create the correct drivetrain system based on that. 
		//If we get something wacky, throw an error
		switch (Constants.currentMode) {
		case REAL:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIOPigeon2(false),
						new ModuleIOSparkMax(0), new ModuleIOSparkMax(1),
						new ModuleIOSparkMax(2), new ModuleIOSparkMax(3));
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			case TANK:
				switch (DriveConstants.robotMotorController) {
				case CTRE_MOTORS:
					drivetrainS = new Tank(new DriveIOTalonFX());
					break;
				case NEO_SPARK_MAX:
				case VORTEX_SPARK_FLEX:
					drivetrainS = new Tank(new DriveIOSparkMax());
					break;
				}
				break;
			case MECANUM:
				switch (DriveConstants.robotMotorController) {
				case CTRE_MOTORS:
					drivetrainS = new CTREMecanumS(new CTREMecanumConstantContainer(
							30, DriveConstants.kFrontLeftDrivePort,
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
				default:
					//10, 11, 12, 13, 80, 7.5,
					drivetrainS = new REVMecanumS(new REVMecanumConstantContainer(10,
							11, 12, 13, 80, 7.5, TrainConstants.kWheelDiameter,
							DriveConstants.kModuleTranslations,
							Units.inchesToMeters(6)));
					break;
				}
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			//Placeholder values
			default:
				throw new IllegalArgumentException(
						"Unknown implementation type, please check DriveConstants.java!");
			}
			
			switch (StateSpaceConstants.Flywheel.motorVendor){
				case CTRE_MOTORS:
				flywheelS = new FlywheelS(new FlywheelIOTalon());

				break;
				default:
				flywheelS = new FlywheelS(new FlywheelIOSpark());
				break;
			}
			break;
		case SIM:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIO() {}, new ModuleIOSim(),
						new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
				break;
			case TANK:
			System.err.println("MAKING");
				drivetrainS = new Tank(new DriveIOSim());
				break;
			default:
				switch (DriveConstants.robotMotorController) {
				case CTRE_MOTORS:
					drivetrainS = new CTREMecanumS(new CTREMecanumConstantContainer(
							30, DriveConstants.kFrontLeftDrivePort,
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
				default:
					//10, 11, 12, 13, 80, 7.5,
					drivetrainS = new REVMecanumS(new REVMecanumConstantContainer(10,
							11, 12, 13, 80, 7.5, TrainConstants.kWheelDiameter,
							DriveConstants.kModuleTranslations,
							Units.inchesToMeters(6)));
					break;
				}
				PPHolonomicDriveController
						.setRotationTargetOverride(this::getRotationTargetOverride);
				break;
			}
			
			flywheelS = new FlywheelS(new FlywheelIOSim());
			armS = new SingleJointedArmS(new SingleJointedArmIOSim());

			break;
		default:
			switch (DriveConstants.driveType) {
			case SWERVE:
				drivetrainS = new Swerve(new GyroIO() {}, new ModuleIO() {},
						new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
				break;
			case TANK:
				drivetrainS = new Tank(new DriveIO() {});
				break;
			case MECANUM:
				throw new IllegalArgumentException(
						"Mecanum does NOT support replay.");
			}
			flywheelS = new FlywheelS(new FlywheelIO(){});
		}
		
		drivetrainS.setDefaultCommand(new DrivetrainC(drivetrainS));
		List<Pair<String, Command>> autoCommands = Arrays.asList(
		//new Pair<String,Command>("AimAtAmp",new AimToPose(drivetrainS, new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(0)))))
		//new Pair<String, Command>("BranchGrabbingGamePiece", new BranchAuto("grabGamePieceBranch",new Pose2d(0,0,new Rotation2d())))
		//new Pair<String, Command>("DriveToAmp",new DriveToPose(drivetrainS, false,new Pose2d(1.9,7.7,new Rotation2d(Units.degreesToRadians(90))))),
		//new Pair<String,Command>("PlayMiiSong", new OrchestraC("mii")),
		//new Pair<String,Command>("SimBot",new SimDefenseBot())
		);
		Pathfinding.setPathfinder(new LocalADStarAK());
		NamedCommands.registerCommands(autoCommands);
		if (Constants.isCompetition) {
			PPLibTelemetry.enableCompetitionMode();
		}
		PathfindingCommand.warmupCommand()
		.finallyDo(() -> RobotContainer.field.getObject("target pose")
				.setPose(new Pose2d(-50, -50, new Rotation2d())))
		.schedule();
		flywheelS.setDefaultCommand(new FlywheelC(flywheelS));
		armS.setDefaultCommand(new SingleJointedArmC(armS));
		elevatorS.setDefaultCommand(new CTREElevatorC(elevatorS));
		doubleJointedArmS.setDefaultCommand(new CTREDoubleJointedArmC(doubleJointedArmS));
		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(field);
		SmartDashboard.putData("Auto Chooser", autoChooser);
		autoChooser.onChange(auto -> {
			try {
				field.getObject("path")
						.setPoses(PathFinder.parseAutoToPose2dList(auto.getName()));
			}
			catch (Exception e) {
				System.err.println("NO FOUND PATH FOR DESIRED AUTO!!");
				field.getObject("path").setPoses(
						new Pose2d[] { new Pose2d(-50, -50, new Rotation2d()),
								new Pose2d(-50.2, -50, new Rotation2d())
				});
			}
		});
		// Configure the trigger bindings
		configureBindings();
		addNTCommands();
	}
	public Optional<Rotation2d> getRotationTargetOverride(){
		// Some condition that should decide if we want to override rotation
		return angleOverrider;
	}
	private void configureBindings() {
		xButtonDrive
				.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest)
						.negate())
				.onTrue(new InstantCommand(() -> drivetrainS.zeroHeading()));  //whileTrue(PathFinder.goToPose(new Pose2d(1.9, 7.7,new Rotation2d(Units.degreesToRadians(90))),DriveConstants.pathConstraints, drivetrainS, false))
		yButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, true, drivetrainS));
		bButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, true, drivetrainS));
		aButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kForward, false, drivetrainS));
		xButtonTest.whileTrue(
				new RunTest(SysIdRoutine.Direction.kReverse, false, drivetrainS));
		//Example Aim To 2024 Amp Pose, Bind to what you need.
		//yButtonDrive.and(aButtonTest.or(bButtonTest).or(xButtonTest).or(yButtonTest).negate()).whileTrue(new AimToPose(drivetrainS,new Pose2d(1.9,7.7, new Rotation2d(Units.degreesToRadians(90)))));
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
		return new double[] {Math.min(drivetrainS.getCurrent(), 200),
      flywheelS.getCurrent(),
			armS.getCurrent(),
			elevatorS.getDrawnCurrentAmps(),
		};
	}
	private static void addNTCommands() {
		SmartDashboard.putData("SystemStatus/AllSystemsCheck", allSystemsCheck());
	 }
	/**
	 * RUN EACH system's test command.
	 * Does NOT run any checks on vision. 
	 * @return a command with all of them in a sequence.
	 */
	public static Command allSystemsCheck() {
	return Commands.sequence(drivetrainS.getRunnableSystemCheckCommand(),flywheelS.getSystemCheckCommand(),armS.getSystemCheckCommand(),elevatorS.getSystemCheckCommand(),doubleJointedArmS.getSystemCheckCommand());
	}
	public static HashMap<String, Double> combineMaps(List<HashMap<String, Double>> maps) {
		HashMap<String, Double> combinedMap = new HashMap<>();

		// Iterate over the list of maps
		for (HashMap<String, Double> map : maps) {
			 combinedMap.putAll(map);
		}

		return combinedMap;
  }

	public static HashMap<String, Double> getAllTemps(){
		// List of HashMaps
		List<HashMap<String, Double>> maps = List.of(drivetrainS.getTemps(),
		flywheelS.getTemps(),
		armS.getTemps(),
		elevatorS.getTemps(),
		doubleJointedArmS.getTemps());

		// Combine all maps
		HashMap<String, Double> combinedMap = combineMaps(maps);
		return combinedMap;
	}
	/**
	 * Checks EACH system's status (DOES NOT RUN THE TESTS)  
	 * @return true if ALL systems were good.
	 */
	public static boolean allSystemsOK() {
		return drivetrainS.getTrueSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& flywheelS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& elevatorS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& armS.getSystemStatus() == SubsystemChecker.SystemStatus.OK
		&& doubleJointedArmS.getSystemStatus() == SubsystemChecker.SystemStatus.OK;
	 }
	public static Collection<ParentDevice> getOrchestraDevices() {
    
		Collection<ParentDevice> devices = new ArrayList<>();
		devices.addAll(drivetrainS.getDriveOrchestraDevices());
		devices.addAll(flywheelS.getOrchestraDevices());
		devices.addAll(elevatorS.getOrchestraDevices());
    devices.addAll(armS.getOrchestraDevices());
		devices.addAll(doubleJointedArmS.getOrchestraDevices());

    return devices;
	}
	public static Subsystem[] getAllSubsystems(){
		Subsystem[] subsystems = new Subsystem[5];
		subsystems[0] = drivetrainS;
    	subsystems[1] = flywheelS;
    	subsystems[2] = elevatorS;
    	subsystems[3] = armS;
    	subsystems[4] = doubleJointedArmS;
		return subsystems;
	}
}
