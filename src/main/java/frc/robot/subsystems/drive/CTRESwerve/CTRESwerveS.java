package frc.robot.subsystems.drive.CTRESwerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.SimGamePiece;
import frc.robot.utils.drive.DriveConstants;

import static edu.wpi.first.units.Units.Volts;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CTRESwerveS extends SwerveDrivetrain implements DrivetrainS {
	private Supplier<Pose2d> pose2dSupplier = () -> {
		return getPose();
	};
	private static final double kSimLoopPeriod = 0.01; // 5 ms
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;
	private final SwerveRequest.ApplyChassisSpeeds AutoRequest = new SwerveRequest.ApplyChassisSpeeds();

	public CTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			double OdometryUpdateFrequency, Telemetry logger,
			SwerveModuleConstants... modules) {
		super(driveTrainConstants, OdometryUpdateFrequency, modules);
		if (Constants.currentMode == Constants.Mode.SIM) {
			startSimThread();
			super.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
			SimGamePiece.setRobotPoseSupplier(pose2dSupplier);
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

	public CTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			Telemetry logger, SwerveModuleConstants... modules) {
		super(driveTrainConstants, modules);
		if (Constants.currentMode == Constants.Mode.SIM) {
			startSimThread();
			super.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
			SimGamePiece.setRobotPoseSupplier(pose2dSupplier);
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

	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get()));
	}

	public void setChassisSpeeds(ChassisSpeeds speeds) {
		AutoRequest.withSpeeds(speeds).apply(m_requestParameters, Modules);
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return m_kinematics.toChassisSpeeds(getState().ModuleStates);
	}

	@Override
	public void resetPose(Pose2d pose) { super.seedFieldRelative(pose); }

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		super.addVisionMeasurement(getPose(), timestamp, estStdDevs);
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

	@Override
	public void stopModules() {
		new SwerveRequest.SwerveDriveBrake().apply(m_requestParameters, Modules);
	}

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

	@Override
	public Command sysIdDynamicTurn(Direction direction) {
		RoutineToApply = SysIdRoutineRotation;
		return RoutineToApply.dynamic(direction);
	}

	@Override
	public Command sysIdQuasistaticTurn(Direction direction) {
		RoutineToApply = SysIdRoutineRotation;
		return RoutineToApply.quasistatic(direction);
	}

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		RoutineToApply = SysIdRoutineSteer;
		return RoutineToApply.dynamic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		RoutineToApply = SysIdRoutineSteer;
		return RoutineToApply.quasistatic(direction);
	}

	@Override
	public void zeroHeading() { super.seedFieldRelative(); }

	@Override
	public Rotation2d getRotation2d() {
	return super.getRotation3d().toRotation2d();
 }
}
