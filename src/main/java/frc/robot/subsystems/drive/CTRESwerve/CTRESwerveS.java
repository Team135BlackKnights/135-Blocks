package frc.robot.subsystems.drive.CTRESwerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

import static edu.wpi.first.units.Units.Volts;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CTRESwerveS extends SwerveDrivetrain implements DrivetrainS {
	private static final double kSimLoopPeriod = 0.01; // 5 ms
	private Notifier m_simNotifier = null; //Checks for updates
	private double m_lastSimTime;
	private double deadband = .1;
	private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();
	private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
	private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
			.withDeadband(TunerConstants.kSpeedAt12VoltsMps * deadband)
			.withRotationalDeadband(
					DriveConstants.kTeleTurningMaxAcceleration * deadband) // Add a 10% deadband
			.withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
	// driving in open loop

	/**
	 * Creates a CTRE Swerve Drivetrain
	 * 
	 * @param driveTrainConstants     Example of this is in TunerConstants.java
	 * @param OdometryUpdateFrequency How often to update the odometry
	 * @param logger                  //Where to log the data
	 * @param modules                 //Which modules to use
	 */
	public CTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			double OdometryUpdateFrequency, Telemetry logger,
			SwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		if (Constants.currentMode == Constants.Mode.SIM) {
			startSimThread();
			super.seedFieldRelative(
					new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
		}
		//Pathplanner declaration
		AutoBuilder.configureHolonomic(() -> this.getState().Pose, // Supplier of current robot pose
				this::seedFieldRelative, // Consumer for seeding pose against auto
				this::getChassisSpeeds, this::setChassisSpeeds, // Consumer of ChassisSpeeds to drive the robot
				new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
						new PIDConstants(10, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
						DriveConstants.kDriveBaseRadius, new ReplanningConfig()),
				() -> DriverStation.getAlliance()
						.orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
				this); // Subsystem for requirements
		super.registerTelemetry(logger::telemeterize);
	}

	/**
	 * Creates a CTRE Swerve Drivetrain. Does not have an updateOdometryFrequency
	 * since this is run in sim.
	 * 
	 * @param driveTrainConstants Example of this is in TunerConstants.java
	 * @param logger              //Where to log the data
	 * @param modules             //Which modules to use
	 */
	public CTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			Telemetry logger, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		if (Constants.currentMode == Constants.Mode.SIM) {
			startSimThread();
			super.seedFieldRelative(
					new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
		}
		AutoBuilder.configureHolonomic(() -> this.getState().Pose, // Supplier of current robot pose
				this::seedFieldRelative, // Consumer for seeding pose against auto
				this::getChassisSpeeds, this::setChassisSpeeds, // Consumer of ChassisSpeeds to drive the robot
				new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
						new PIDConstants(10, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
						DriveConstants.kDriveBaseRadius, new ReplanningConfig()),
				() -> DriverStation.getAlliance()
						.orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
				this); // Subsystem for requirements
		super.registerTelemetry(logger::telemeterize);
	}

	@Override
	public void applyRequest() {
		drive.withVelocityX(-RobotContainer.driveController.getLeftY()
				* TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
				// negative Y (forward)
				.withVelocityY(-RobotContainer.driveController.getLeftX()
						* TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
				.withRotationalRate(-RobotContainer.driveController.getRightX()
						* DriveConstants.kTeleTurningMaxAcceleration)
				.withDeadband(TunerConstants.kSpeedAt12VoltsMps * deadband)
				.withRotationalDeadband(
						DriveConstants.kTeleTurningMaxAcceleration * deadband) // Drive counterclockwise with negative X (left)
				.apply(m_requestParameters, Modules);
	}

	/**
	 * Set the ChassisSpeeds to the drivetrain
	 * 
	 * @param speeds The speeds to be set
	 */
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		AutoRequest.withSpeeds(speeds.times(2)).apply(m_requestParameters, Modules);
	}

	/**
	 * Get the ChassisSpeeds of the drivetrain
	 * 
	 * @return the drivetrain ChassisSpeeds
	 */
	@Override
	public ChassisSpeeds getChassisSpeeds() {
		if (super.getState().ModuleStates == null)
			return new ChassisSpeeds(0, 0, 0);
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
	}

	/**
	 * Reset the pose of the robot (it thinks the pose it's at when it's reset is
	 * the starting pose)
	 */
	@Override
	public void resetPose(Pose2d pose) { super.seedFieldRelative(pose); }

	/**
	 * Adds a vision measurement to the poseEstimator
	 * 
	 * @param pose      the pose the camera outputs
	 * @param timestamp the timestamp of when the measurement was taken
	 */
	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		super.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Pose2d getPose() { return super.getState().Pose; }

	private void startSimThread() {
		m_lastSimTime = Utils.getCurrentTimeSeconds();
		/* Run simulation at a faster rate so PID gains behave more reasonably */
		m_simNotifier = new Notifier(() -> {
			final double currentTime = Utils.getCurrentTimeSeconds();
			double deltaTime = currentTime - m_lastSimTime;
			m_lastSimTime = currentTime;
			/* use the measured time delta, get battery voltage from WPILib */
			updateSimState(deltaTime, RobotController.getBatteryVoltage());
		});
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}

	/**
	 * Stops the modules
	 */
	@Override
	public void stopModules() { brake.apply(m_requestParameters, Modules); }

	private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveRotation RotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
	private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
	/* Use one of these sysidroutines for your particular test */
	private SysIdRoutine SysIdRoutineTranslation = new SysIdRoutine(
			new SysIdRoutine.Config(null, Volts.of(4), null,
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism(
					(volts) -> setControl(
							TranslationCharacterization.withVolts(volts)),
					null, this));
	private final SysIdRoutine SysIdRoutineRotation = new SysIdRoutine(
			new SysIdRoutine.Config(null, Volts.of(4), null,
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism(
					(volts) -> setControl(RotationCharacterization.withVolts(volts)),
					null, this));
	private final SysIdRoutine SysIdRoutineSteer = new SysIdRoutine(
			new SysIdRoutine.Config(null, Volts.of(7), null,
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism(
					(volts) -> setControl(SteerCharacterization.withVolts(volts)),
					null, this));
	/* Change this to the sysid routine you want to test */
	private SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

	/**
	 * Commands for SysID, see the WPILIB SysID documentation for details at
	 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
	 */
	@Override
	public Command sysIdDynamicTurn(Direction direction) {
		RoutineToApply = SysIdRoutineRotation;
		return RoutineToApply.dynamic(direction);
	}

	/**
	 * Commands for SysID, see the WPILIB SysID documentation for details at
	 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
	 */
	@Override
	public Command sysIdQuasistaticTurn(Direction direction) {
		RoutineToApply = SysIdRoutineRotation;
		return RoutineToApply.quasistatic(direction);
	}

	/**
	 * Commands for SysID, see the WPILIB SysID documentation for details at
	 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
	 */
	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		RoutineToApply = SysIdRoutineSteer;
		return RoutineToApply.dynamic(direction);
	}

	/**
	 * Commands for SysID, see the WPILIB SysID documentation for details at
	 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html
	 */
	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		RoutineToApply = SysIdRoutineSteer;
		return RoutineToApply.quasistatic(direction);
	}

	/**
	 * Zero the heading of the drivetrain (the rotation it's at is viewed as its
	 * starting rotation)
	 */
	@Override
	public void zeroHeading() { super.seedFieldRelative(); }

	/**
	 * Get the rotation2d of the robot
	 * 
	 * @return The rotation2d of the robot, goober
	 */
	@Override
	public Rotation2d getRotation2d() {
		return super.getRotation3d().toRotation2d();
	}

	@Override
	public boolean isConnected() {
		return super.getPigeon2().getFault_Hardware().getValue();
	}

	@Override
	public double getYawVelocity() {
		if (Constants.currentMode == Constants.Mode.REAL) {
			return Units.degreesToRadians(super.getPigeon2()
					.getAngularVelocityZWorld().getValueAsDouble());
		}
		return super.getState().speeds.omegaRadiansPerSecond;
	}

	@Override
	public Twist2d getFieldVelocity() {
		ChassisSpeeds m_ChassisSpeeds = super.getState().speeds;
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(),
				m_ChassisSpeeds.omegaRadiansPerSecond);
	}

	@Override
	public void changeDeadband(double newDeadband) {
		deadband = newDeadband;
		drive.Deadband = TunerConstants.kSpeedAt12VoltsMps * deadband;
		drive.RotationalDeadband = DriveConstants.kTeleTurningMaxAcceleration
				* deadband;
	}
}
