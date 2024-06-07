package frc.robot.subsystems.drive.REVSwerve;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.drive.REVSwerve.SwerveModules.REVSwerveModule;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.HashMap;

import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;
import frc.robot.utils.drive.Position;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class REVSwerveS extends SubsystemBase implements DrivetrainS {
	private static Translation2d[] kModuleTranslations;
	private static double kMaxSpeedMetersPerSecond, kDriveBaseRadius;
	private static REVModuleConstantContainer[] REVModuleConstantContainers = new REVModuleConstantContainer[4];
	private static SwerveDriveKinematics kDriveKinematics;
	private HashMap<ModulePosition, REVSwerveModule> m_swerveModules = new HashMap<>();
	private static AHRS gyro = new AHRS(Port.kUSB1);
	//Whether the robot should be field oriented
	//Whether the swerve drivetrain should be taken over by our auto drive to note feature (in the Vision branch)
	public Pose2d robotPosition = new Pose2d(0, 0, getRotation2d());
	Field2d robotField = new Field2d();
	private Twist2d fieldVelocity = new Twist2d();
	// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
	ChassisSpeeds m_ChassisSpeeds;
	static Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
	static Vector<N3> visionStdDevs = VecBuilder.fill(1, 1, 1);
	public SwerveDrivePoseEstimator poseEstimator;
	Position<SwerveModulePosition[]> m_modulePositions;
	private double m_simYaw;
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(8);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (REVSwerveModule module : m_swerveModules.values())
					module.setTurningTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (REVSwerveModule module : m_swerveModules.values())
					module.setDriveTest(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));

	/**
	 * Creates a new REVSwerve Drivetrain with 8 CANSparkMaxes or 8
	 * CANSparkFlexes
	 * 
	 * @param maxSpeed                 the max speed of the drivetrain
	 * @param driveBaseRadius          the radius of the drivetrain
	 * @param moduleConstantContainers the moduleConstantContainers for each
	 *                                    module as an array of {frontLeft,
	 *                                    frontRight, backLeft, backRight}
	 * @see REVModuleConstantContainer
	 */
	public REVSwerveS(REVModuleConstantContainer[] moduleConstantContainers,
			double maxSpeed, double driveBaseRadius) {
		REVModuleConstantContainers = moduleConstantContainers;
		initalizeModules();
		kModuleTranslations = new Translation2d[] {
				moduleConstantContainers[0].getTranslation2d(),
				moduleConstantContainers[1].getTranslation2d(),
				moduleConstantContainers[2].getTranslation2d(),
				moduleConstantContainers[3].getTranslation2d()
		};
		kDriveKinematics = new SwerveDriveKinematics(kModuleTranslations);
		m_ChassisSpeeds = kDriveKinematics.toChassisSpeeds(getModuleStates());
		m_modulePositions = getPositionsWithTimestamp(getModulePositions());
		poseEstimator = new SwerveDrivePoseEstimator(kDriveKinematics,
				getRotation2d(), m_modulePositions.getPositions(), robotPosition,
				stateStdDevs, visionStdDevs);
		kMaxSpeedMetersPerSecond = maxSpeed;
		kDriveBaseRadius = driveBaseRadius;
		// Waits for the RIO to finishing booting
		new Thread(() -> {
			try {
				Thread.sleep(1000);
				zeroHeading();
				for (REVSwerveModule module : m_swerveModules.values())
					module.resetEncoders();
				//limelight.getEntry("pipeline").setNumber(1);
			}
			catch (Exception e) {
			}
		}).start();
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), () -> Robot.isRed, this // Reference to this subsystem to set requirements
		);
	}

	/**
	 * Returns a command that will execute a quasistatic test in the given
	 * direction.
	 *
	 * @param direction The direction (forward or reverse) to run the test in
	 */
	@Override
	public Command sysIdQuasistaticTurn(SysIdRoutine.Direction direction) {
		return sysIdRoutineTurn.quasistatic(direction);
	}

	/**
	 * Returns a command that will execute a dynamic test in the given direction.
	 *
	 * @param direction The direction (forward or reverse) to run the test in
	 */
	@Override
	public Command sysIdDynamicTurn(SysIdRoutine.Direction direction) {
		return sysIdRoutineTurn.dynamic(direction);
	}

	@Override
	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysIdRoutineDrive.quasistatic(direction);
	}

	//public static boolean autoLock = false;
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

	public SwerveModulePosition[] getModulePositions() {
		return new SwerveModulePosition[] {
				m_swerveModules.get(ModulePosition.FRONT_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.FRONT_RIGHT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_LEFT).getPosition(),
				m_swerveModules.get(ModulePosition.BACK_RIGHT).getPosition()
		};
	}

	private void initalizeModules() {
		m_swerveModules.clear();
		REVSwerveModule front_left = new REVSwerveModule(
				REVModuleConstantContainers[0]);
		REVSwerveModule front_right = new REVSwerveModule(
				REVModuleConstantContainers[1]);
		REVSwerveModule back_left = new REVSwerveModule(
				REVModuleConstantContainers[2]);
		REVSwerveModule back_right = new REVSwerveModule(
				REVModuleConstantContainers[3]);
		for (ModulePosition position : ModulePosition.values()) {
			if (position.name() == "FRONT_LEFT") {
				m_swerveModules.put(position, front_left);
			} else if (position.name() == "FRONT_RIGHT") {
				m_swerveModules.put(position, front_right);
			} else if (position.name() == "BACK_LEFT") {
				m_swerveModules.put(position, back_left);
			} else if (position.name() == "BACK_RIGHT") {
				m_swerveModules.put(position, back_right);
			}
		}
		SmartDashboard.putData("Swerve Drive", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle",
						() -> m_swerveModules.get(ModulePosition.FRONT_LEFT).getHeadingRotation2d().getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity",
						() -> m_swerveModules.get(ModulePosition.FRONT_LEFT).getDriveVelocity(), null);
				builder.addDoubleProperty("Front Right Angle",
						() -> m_swerveModules.get(ModulePosition.FRONT_RIGHT).getHeadingRotation2d().getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity",
						() -> m_swerveModules.get(ModulePosition.FRONT_RIGHT).getDriveVelocity(), null);
				builder.addDoubleProperty("Back Left Angle",
						() -> m_swerveModules.get(ModulePosition.BACK_LEFT).getHeadingRotation2d().getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity",
						() -> m_swerveModules.get(ModulePosition.BACK_LEFT).getDriveVelocity(), null);
				builder.addDoubleProperty("Back Right Angle",
						() -> m_swerveModules.get(ModulePosition.BACK_RIGHT).getHeadingRotation2d().getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity",
						() -> m_swerveModules.get(ModulePosition.BACK_RIGHT).getDriveVelocity(), null);
				builder.addDoubleProperty("Robot Angle",
						() -> Units.degreesToRadians(getHeading()), null);
			}
		});
	}

	@Override
	public void zeroHeading() {
		debounce = 0;
		gyro.reset();
		poseEstimator.resetPosition(gyro.getRotation2d(),
				m_modulePositions.getPositions(), robotPosition);
	}

	public static double getHeading() {
		return -1 * Math.IEEEremainder(gyro.getAngle() + (Robot.isRed ? 180 : 0),
				360); //modulus
	}

	@Override
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(getHeading());
	}
	//public static boolean getAutoLock() { return autoLock; }

	@Override
	public void periodic() {
		if (Constants.currentMode == Mode.SIM) {
			ChassisSpeeds chassisSpeed = kDriveKinematics
					.toChassisSpeeds(getModuleStates());
			m_simYaw += chassisSpeed.omegaRadiansPerSecond * 0.02;
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(
					SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// NavX expects clockwise positive, but sim outputs clockwise negative
			angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_simYaw), 360));
			//m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
		}
		//puts values to smartDashboard
		//SmartDashboard.putBoolean("Auto Lock", autoLock);
		SmartDashboard.putNumber("Robot Heading (getPose)",
				getPose().getRotation().getDegrees());
		m_modulePositions = getPositionsWithTimestamp(getModulePositions());
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		m_ChassisSpeeds = kDriveKinematics.toChassisSpeeds(getModuleStates());
		robotPosition = poseEstimator.updateWithTime(
				m_modulePositions.getTimestamp(), getRotation2d(),
				m_modulePositions.getPositions());
		for (REVSwerveModule module : m_swerveModules.values()) {
			var modulePositionFromChassis = kModuleTranslations[module
					.getModuleNumber()].rotateBy(getRotation2d())
							.plus(getPoseMeters().getTranslation());
			module.setModulePose(new Pose2d(modulePositionFromChassis,
					module.getHeadingRotation2d().plus(getRotation2d())));
		}
		robotField.setRobotPose(getPose());
		SmartDashboard.putData(robotField);
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
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omegaRadiansPerSecond);
	}

	@AutoLogOutput(key = "Odometry/Robot")
	@Override
	public Pose2d getPose() { return robotPosition; }

	@Override
	public ChassisSpeeds getChassisSpeeds() { return m_ChassisSpeeds; }

	@Override
	public void resetPose(Pose2d pose) {
		// LIST MODULES IN THE SAME EXACT ORDER USED WHEN DECLARING SwerveDriveKinematics
		poseEstimator.resetPosition(getRotation2d(),
				m_modulePositions.getPositions(), pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public void stopModules() {
		for (REVSwerveModule module : m_swerveModules.values())
			module.stop();
	}

	public Pose2d getPoseMeters() {
		return poseEstimator.getEstimatedPosition();
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speed) {
		SwerveModuleState[] moduleStates = kDriveKinematics
				.toSwerveModuleStates(speed);
		SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
				kMaxSpeedMetersPerSecond);
		for (REVSwerveModule module : m_swerveModules.values()) {
			module.setDesiredState(moduleStates[module.getModuleNumber()]);
		}
		Logger.recordOutput("Swerve/Display/Target Swerve Module States",
				moduleStates);
	}

	@Override
	public boolean isConnected() { return gyro.isConnected(); }

	public void navXDisconnectProtocol() {
		if (gyro.isConnected() && debounce == 0) {
			DriveConstants.fieldOriented = true;
			return;
		} else {
			debounce = -1;
			DriveConstants.fieldOriented = false;
		}
	}

	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; //?
	}

	@Override
	public Twist2d getFieldVelocity() { return fieldVelocity; }
}
