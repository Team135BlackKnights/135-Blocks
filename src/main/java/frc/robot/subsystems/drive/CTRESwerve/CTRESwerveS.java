package frc.robot.subsystems.drive.CTRESwerve;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;
import java.util.HashMap;


/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class CTRESwerveS extends SwerveDrivetrain implements DrivetrainS {
	private static final double kSimLoopPeriod = 0.01; // 5 ms
	private Notifier m_simNotifier = null; //Checks for updates
	private double m_lastSimTime, deadband = .1, last_world_linear_accel_x, last_world_linear_accel_y;
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
		super.registerTelemetry(logger::telemeterize);
		SwerveModuleState[] moduleStates = super.m_moduleStates;
		SmartDashboard.putData("Swerve Drive", new Sendable() {
  			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle",
						() -> moduleStates[0].angle.getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity",
						() -> moduleStates[0].speedMetersPerSecond, null);
				builder.addDoubleProperty("Front Right Angle",
						() -> moduleStates[1].angle.getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity",
						() -> moduleStates[1].speedMetersPerSecond, null);
				builder.addDoubleProperty("Back Left Angle",
						() -> moduleStates[2].angle.getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity",
						() -> moduleStates[2].speedMetersPerSecond, null);
				builder.addDoubleProperty("Back Right Angle",
						() -> moduleStates[3].angle.getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity",
						() -> moduleStates[3].speedMetersPerSecond, null);
				builder.addDoubleProperty("Robot Angle",
						() -> getPose().getRotation()
								.plus(new Rotation2d(Units.degreesToRadians(0)))
								.getRadians(),
						null); //🥥🥥🥥
			}
		});
	}
	@Override
	public boolean[] isSkidding(){
		return calculateSkidding();
	}
	public boolean[] calculateSkidding() {
		SwerveModuleState[] moduleStates = super.getState().ModuleStates;
		ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
		// Step 1: Create a ChassisSpeeds object with solely the rotation component
		ChassisSpeeds rotationOnlySpeeds = new ChassisSpeeds(0.0, 0.0,
			currentChassisSpeeds.omegaRadiansPerSecond+.05);
		double[] xComponentList = new double[4];
		double[] yComponentList = new double[4];
		// Step 2: Convert it into module states with kinematics
		SwerveModuleState[] rotationalStates = m_kinematics
				.toSwerveModuleStates(rotationOnlySpeeds);
		// Step 3: Subtract the rotational states from the module states and calculate the magnitudes
		for (int i = 0; i < moduleStates.length; i++) {
			double deltaX = moduleStates[i].speedMetersPerSecond
					* Math.cos(moduleStates[i].angle.getRadians())
					- rotationalStates[i].speedMetersPerSecond
							* Math.cos(rotationalStates[i].angle.getRadians());
			double deltaY = moduleStates[i].speedMetersPerSecond
					* Math.sin(moduleStates[i].angle.getRadians())
					- rotationalStates[i].speedMetersPerSecond
							* Math.sin(rotationalStates[i].angle.getRadians());
			xComponentList[i] = deltaX; 
			yComponentList[i] = deltaY;
		}
		Arrays.sort(xComponentList);
		Arrays.sort(yComponentList);
		SmartDashboard.putNumberArray("Module Skid X", xComponentList);
		SmartDashboard.putNumberArray("Module Skid Y", yComponentList);

		double deltaMedianX = (xComponentList[1] + xComponentList[2]) / 2;
		double deltaMedianY = (yComponentList[1] + yComponentList[2]) / 2;
		SmartDashboard.putNumber("Skid X Median", deltaMedianX);
		SmartDashboard.putNumber("Skid Y Median", deltaMedianY);

		boolean[] areModulesSkidding = new boolean[4];
		for (int i = 0; i < 4; i++){
			double deltaX = xComponentList[i];
			double deltaY = yComponentList[i];
			if (Math.abs(deltaX - deltaMedianX) > DriveConstants.SKID_THRESHOLD || Math.abs(deltaY - deltaMedianY) > DriveConstants.SKID_THRESHOLD){
				areModulesSkidding[i] = true;
			}else{
				areModulesSkidding[i] = false;
			}
		}
		SmartDashboard.putBooleanArray("Module Skids", areModulesSkidding);
		return areModulesSkidding;
	}
	public boolean collisionDetected() {
		double curr_world_linear_accel_x = super.m_pigeon2.getAccelerationX().getValueAsDouble();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = super.m_pigeon2.getAccelerationY().getValueAsDouble();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			return true;
		}
		return false;
	}

	@Override
	public void applyRequest() {
		double rotationRate = 0, rotationDeadband = 0;
		if (RobotContainer.angularSpeed !=0){
			rotationRate = RobotContainer.angularSpeed*2;
			rotationDeadband = 0.01;
		}else{
			rotationRate = -RobotContainer.driveController.getRightX() * DriveConstants.kTeleTurningMaxAcceleration;
			rotationDeadband = DriveConstants.kTeleTurningMaxAcceleration * deadband;
		}
		drive.withVelocityX(-RobotContainer.driveController.getLeftY()
				* TunerConstants.kSpeedAt12VoltsMps) // Drive forward with
				// negative Y (forward)
				.withVelocityY(-RobotContainer.driveController.getLeftX()
						* TunerConstants.kSpeedAt12VoltsMps) // Drive left with negative X (left)
				.withRotationalRate(rotationRate)
				.withDeadband(TunerConstants.kSpeedAt12VoltsMps * deadband)
				.withRotationalDeadband(
						rotationDeadband) // Drive counterclockwise with negative X (left)
				.apply(m_requestParameters, Modules);
	}

	/**
	 * Set the ChassisSpeeds to the drivetrain
	 * 
	 * @param speeds The speeds to be set
	 */
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		if (RobotContainer.currentPath == "DRIVETOPOSE"){
			AutoRequest.withSpeeds(speeds.times(2)).apply(m_requestParameters, Modules);
		}else{
			AutoRequest.withSpeeds(speeds).apply(m_requestParameters, Modules);
		}
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
	@Override
	public double getCurrent() {
		return Math.abs(super.Modules[0].getDriveMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[0].getSteerMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[1].getDriveMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[1].getSteerMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[2].getDriveMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[2].getSteerMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[3].getDriveMotor().getStatorCurrent().getValueAsDouble())
				+ Math.abs(super.Modules[3].getSteerMotor().getStatorCurrent().getValueAsDouble());
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

	@Override
	public HashMap<String, Double> getTemps() { 
	throw new UnsupportedOperationException("Unimplemented method 'getTemps'"); }

	@Override
	public boolean isCollisionDetected() {
	throw new UnsupportedOperationException("Unimplemented method 'isCollisionDetected'"); }
}
