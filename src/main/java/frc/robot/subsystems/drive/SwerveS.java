package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import frc.robot.Constants.Mode;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.HashMap;
import java.util.Map;

import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.SwerveConstants.ModulePosition;
import java.util.function.Supplier;
import frc.robot.utils.SimGamePiece;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* uncomment for autolock
 * import edu.wpi.first.math.controller.PIDController;
 * import edu.wpi.first.wpilibj2.command.InstantCommand;
 */

public class SwerveS extends SubsystemBase {
	//This function returns pose 
	private Supplier<Pose2d> pose2dSupplier = () -> {
		return getPose();
	};
	private final static HashMap<ModulePosition, SwerveModule> m_swerveModules = new HashMap<>(
			Map.of(ModulePosition.FRONT_LEFT,
					new SwerveModule(DriveConstants.kFrontLeftDrivePort,
							DriveConstants.kFrontLeftTurningPort,
							DriveConstants.kFrontLeftDriveReversed,
							DriveConstants.kFrontLeftTurningReversed,
							DriveConstants.kFrontLeftAbsEncoderPort,
							DriveConstants.kFrontLeftAbsEncoderOffsetRad,
							DriveConstants.kFrontLeftAbsEncoderReversed,
							DriveConstants.SwerveConstants.frontLeftDriveKpKsKvKa,
							DriveConstants.SwerveConstants.overallTurnkPkSkVkAkD),
					ModulePosition.FRONT_RIGHT,
					new SwerveModule(DriveConstants.kFrontRightDrivePort,
							DriveConstants.kFrontRightTurningPort,
							DriveConstants.kFrontRightDriveReversed,
							DriveConstants.kFrontRightTurningReversed,
							DriveConstants.kFrontRightAbsEncoderPort,
							DriveConstants.kFrontRightAbsEncoderOffsetRad,
							DriveConstants.kFrontRightAbsEncoderReversed,
							DriveConstants.SwerveConstants.frontRightDriveKpKsKvKa,
							DriveConstants.SwerveConstants.overallTurnkPkSkVkAkD),
					ModulePosition.BACK_LEFT,
					new SwerveModule(DriveConstants.kBackLeftDrivePort,
							DriveConstants.kBackLeftTurningPort,
							DriveConstants.kBackLeftDriveReversed,
							DriveConstants.kBackLeftTurningReversed,
							DriveConstants.kBackLeftAbsEncoderPort,
							DriveConstants.kBackLeftAbsEncoderOffsetRad,
							DriveConstants.kBackLeftAbsEncoderReversed,
							DriveConstants.SwerveConstants.backLeftDriveKpKsKvKa,
							DriveConstants.SwerveConstants.overallTurnkPkSkVkAkD),
					ModulePosition.BACK_RIGHT,
					new SwerveModule(DriveConstants.kBackRightDrivePort,
							DriveConstants.kBackRightTurningPort,
							DriveConstants.kBackRightDriveReversed,
							DriveConstants.kBackRightTurningReversed,
							DriveConstants.kBackRightAbsEncoderPort,
							DriveConstants.kBackRightAbsEncoderOffsetRad,
							DriveConstants.kBackRightDriveReversed,
							DriveConstants.SwerveConstants.backRightDriveKpKsKvKa,
							DriveConstants.SwerveConstants.overallTurnkPkSkVkAkD)));

	private static AHRS gyro = new AHRS(Port.kUSB1);
	//Whether the robot should be field oriented
	public static boolean fieldOriented = true,
	//Whether the swerve drivetrain should be taken over by our auto drive to note feature (in the Vision branch)
	takeOver = false;
	static Pose2d robotPosition = new Pose2d(0, 0, getRotation2d());

	Field2d robotField = new Field2d();
	// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
	ChassisSpeeds m_ChassisSpeeds = DriveConstants.kDriveKinematics
			.toChassisSpeeds(getModuleStates());
	static Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
	static Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);
	public static SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			DriveConstants.kDriveKinematics, getRotation2d(),
			getModulePositions(), robotPosition, stateStdDevs, visionStdDevs);
			
	SwerveModulePosition[] m_modulePositions = getModulePositions();
	private double m_simYaw;

	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(8);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState", state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> 
			{
				for (SwerveModule module : m_swerveModules.values())
					module.setTurningTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
			, this));

	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState", state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> 
			{
				for (SwerveModule module : m_swerveModules.values())
					module.setDriveTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
			, this));

	/**
	 * Returns a command that will execute a quasistatic test in the given
	 * direction.
	 *
	 * @param direction The direction (forward or reverse) to run the test in
	 */
	public Command sysIdQuasistaticTurn(SysIdRoutine.Direction direction) {
		return sysIdRoutineTurn.quasistatic(direction);
	}

	/**
	 * Returns a command that will execute a dynamic test in the given direction.
	 *
	 * @param direction The direction (forward or reverse) to run the test in
	 */
	public Command sysIdDynamicTurn(SysIdRoutine.Direction direction) {
		return sysIdRoutineTurn.dynamic(direction);
	}

	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.quasistatic(direction);
	}

	//public static boolean autoLock = false;
	public static boolean redIsAlliance = true;
	//private static double kP, kI, kD, kDistanceMultipler;
	//public PIDController autoLockController; //sadly cannot be system Id'd

	//so that the navXDisconnect command doesn't start twice
	int debounce = 0;

	public SwerveModuleState[] getModuleStates() {
		return new SwerveModuleState[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getState(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getState(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getState(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getState()
		};
	}

	public static SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
		};
	}

	public SwerveS() {
		// Waits for the RIO to finishing booting
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				zeroHeading();
				for (SwerveModule module : m_swerveModules.values())
					module.resetEncoders();
				//limelight.getEntry("pipeline").setNumber(1);
			}
			catch (Exception e) {
			}
		}).start();
		AutoBuilder.configureHolonomic(SwerveS::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), () -> Robot.isRed, this // Reference to this subsystem to set requirements
		);
		
		/*kP = DriveConstants.kP;
		kI = DriveConstants.kI;
		kD = DriveConstants.kD;
		kDistanceMultipler = DriveConstants.kDistanceMultipler;
		SmartDashboard.putNumber("P Gain AutoLock", kP);
		SmartDashboard.putNumber("I Gain AutoLock", kI);
		SmartDashboard.putNumber("D Gain AutoLock", kD);
		SmartDashboard.putNumber("Distance AutoLock", kDistanceMultipler);
		autoLockController = new PIDController(kP, kI, kD);*/
		if (Constants.currentMode == Constants.Mode.SIM){
			SimGamePiece.setRobotPoseSupplier(pose2dSupplier);
		}
	}

	public void zeroHeading() {
		debounce = 0;
		gyro.reset();
	}

	public static double getHeading() {
		return -1 * Math
				.IEEEremainder(gyro.getAngle() + (Robot.isRed ? 180 : 0), 360); //modulus
	}

	public static Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}

	//public static boolean getAutoLock() { return autoLock; }

	@Override
	public void periodic() {
		if (Constants.currentMode == Mode.SIM) {
			ChassisSpeeds chassisSpeed = DriveConstants.kDriveKinematics
					.toChassisSpeeds(getModuleStates());
			m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(
					SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// NavX expects clockwise positive, but sim outputs clockwise negative
			angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_simYaw), 360));
			//m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
			SimGamePiece.updateStates(); //update position of gamePieces
		}
		//puts values to smartDashboard
		//SmartDashboard.putBoolean("Auto Lock", autoLock);
		SmartDashboard.putNumber("Robot Heading (getPose)",
				getPose().getRotation().getDegrees());
		/* 
		double p = SmartDashboard.getNumber("P Gain AutoLock",
				DriveConstants.kP);
		double i = SmartDashboard.getNumber("I Gain AutoLock",
				DriveConstants.kI);
		double d = SmartDashboard.getNumber("D Gain AutoLock",
				DriveConstants.kD);
		double distance = SmartDashboard.getNumber("Distance AutoLock",
				DriveConstants.kDistanceMultipler);
		if ((p != kP)) {
			autoLockController.setP(p);
			kP = p;
		}
		if ((i != kI)) {
			autoLockController.setI(i);
			kI = i;
		}
		if ((d != kD)) {
			autoLockController.setD(d);
			kD = d;
		}
		if ((distance != kDistanceMultipler)) {
			kDistanceMultipler = distance;
		}
		autoLockController.setP(kP / Math.abs((1 +
				(DriveConstants.kDistanceMultipler * getPoseMeters().getX()))));
		// larger the distance, lower the P so not crazy
		*/
		m_modulePositions = getModulePositions();
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		m_ChassisSpeeds = DriveConstants.kDriveKinematics
				.toChassisSpeeds(getModuleStates());
		robotPosition = poseEstimator.update(getRotation2d(), m_modulePositions);
		for (SwerveModule module : m_swerveModules.values()) {
			var modulePositionFromChassis = DriveConstants.kModuleTranslations[module
					.getModuleNumber()].rotateBy(getRotation2d())
							.plus(getPoseMeters().getTranslation());
			module.setModulePose(new Pose2d(modulePositionFromChassis,
					module.getHeadingRotation2d().plus(getRotation2d())));
		}
		robotField.setRobotPose(getPose());
		SmartDashboard.putData(robotField);
		//SmartDashboard.putData(CameraS.getEstimatedField());
		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/CurrentPose", pose);
			robotField.setRobotPose(pose);
		});
		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
			robotField.getObject("target pose").setPose(pose);
		});
		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			Logger.recordOutput("Odometry/Trajectory",
					poses.toArray(new Pose2d[poses.size()]));
			robotField.getObject("path").setPoses(poses);
		});
		navXDisconnectProtocol();
		Logger.recordOutput("Swerve/Display/Actual Swerve Module States",
				getModuleStates());
		Logger.recordOutput("Swerve/Display/Rotation",
				getRotation2d().getDegrees());
	}

	@AutoLogOutput(key = "Odometry/Robot")
	public static Pose2d getPose() { return robotPosition; }

	public ChassisSpeeds getChassisSpeeds() { return m_ChassisSpeeds; }

	

	public void resetPose(Pose2d pose) {
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		poseEstimator.resetPosition(getRotation2d(), m_modulePositions, pose);
	}

	public void stopModules() {
		for (SwerveModule module : m_swerveModules.values())
			module.stop();
	}

	/*public void toggleAutoLock() {
		autoLockController.reset();
		autoLock = !autoLock;
	}

	public InstantCommand toggleAutoLockCommand() {
		return new InstantCommand(this::toggleAutoLock, this);
	}*/

	public Pose2d getPoseMeters() {
		return poseEstimator.getEstimatedPosition();
	}

	public void setChassisSpeeds(ChassisSpeeds speed) {
		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxSpeedMetersPerSecond);
		for (SwerveModule module : m_swerveModules.values()) {
			module.setDesiredState(moduleStates[module.getModuleNumber()]);
		}
		Logger.recordOutput("Swerve/Display/Target Swerve Module States",
				moduleStates);
	}

	public void navXDisconnectProtocol() {
		if (gyro.isConnected() && debounce == 0) {
			fieldOriented = true;
			return;
		} else {
			debounce = -1;
			fieldOriented = false;
		}
	}
}
