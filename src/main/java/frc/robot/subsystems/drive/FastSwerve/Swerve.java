// This system was repurposed from the 6328 Swerve Example, and fed into our
// bigger system.
package frc.robot.subsystems.drive.FastSwerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.LocalADStarAK;
import frc.robot.utils.selfCheck.SelfChecking;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemChecker implements DrivetrainS {
	private static final double DRIVE_BASE_RADIUS = Math.hypot(
			DriveConstants.kChassisWidth / 2.0,
			DriveConstants.kChassisLength / 2.0);
	private static final double MAX_ANGULAR_SPEED = DriveConstants.kMaxSpeedMetersPerSecond
			/ DRIVE_BASE_RADIUS;
	static final Lock odometryLock = new ReentrantLock();
	private final GyroIO gyroIO;
	private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
	private final Module[] modules = new Module[4]; // FL, FR, BL, BR
	private final SysIdRoutine sysId;
	private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
			getModuleTranslations());
	private Rotation2d rawGyroRotation = new Rotation2d();
	private SwerveModulePosition[] lastModulePositions = // For delta tracking
			new SwerveModulePosition[] { new SwerveModulePosition(),
					new SwerveModulePosition(), new SwerveModulePosition(),
					new SwerveModulePosition()
			};
	private SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
			kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
	private boolean collisionDetected;
	private boolean[] isSkidding;
	private Twist2d fieldVelocity;
	private int debounce = 0;
	public Swerve(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO,
			ModuleIO blModuleIO, ModuleIO brModuleIO) {
		this.gyroIO = gyroIO;
		modules[0] = new Module(flModuleIO, 0);
		modules[1] = new Module(frModuleIO, 1);
		modules[2] = new Module(blModuleIO, 2);
		modules[3] = new Module(brModuleIO, 3);
		// Start threads (no-op for each if no signals have been created)
		PhoenixOdometryThread.getInstance().start();
		SparkMaxOdometryThread.getInstance().start();
		// Configure AutoBuilder for PathPlanner
		AutoBuilder.configureHolonomic(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setChassisSpeeds,
				new HolonomicPathFollowerConfig(
						DriveConstants.kMaxSpeedMetersPerSecond, DRIVE_BASE_RADIUS,
						new ReplanningConfig(true, true)),
				() -> Robot.isRed, this);
		Pathfinding.setPathfinder(new LocalADStarAK());
		PathPlannerLogging.setLogActivePathCallback((activePath) -> {
			Logger.recordOutput("Odometry/Trajectory",
					activePath.toArray(new Pose2d[activePath.size()]));
		});
		PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
			Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
		});
		// Configure SysId
		sysId = new SysIdRoutine(
				new SysIdRoutine.Config(null, null, null,
						(state) -> Logger.recordOutput("Drive/SysIdState",
								state.toString())),
				new SysIdRoutine.Mechanism((voltage) -> {
					for (int i = 0; i < 4; i++) {
						modules[i].runCharacterization(voltage.in(Volts));
					}
				}, null, this));
		registerSelfCheckHardware();
	}

	public void periodic() {
		DrivetrainS.super.periodic();
		odometryLock.lock(); // Prevents odometry updates while reading data
		gyroIO.updateInputs(gyroInputs);
		for (var module : modules) {
			module.updateInputs();
		}
		odometryLock.unlock();
		Logger.processInputs("Drive/Gyro", gyroInputs);
		if (debounce ==1 && isConnected()){
			poseEstimator.resetPosition(gyroInputs.yawPosition, lastModulePositions,
			getPose());
			debounce = 0;
		}
		for (var module : modules) {
			module.periodic();
		}
		// Stop moving when disabled
		if (DriverStation.isDisabled()) {
			for (var module : modules) {
				module.stop();
			}
		}
		// Log empty setpoint states when disabled
		if (DriverStation.isDisabled()) {
			Logger.recordOutput("SwerveStates/Setpoints",
					new SwerveModuleState[] {});
			Logger.recordOutput("SwerveStates/SetpointsOptimized",
					new SwerveModuleState[] {});
		}
		// Update odometry
		double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
		int sampleCount = sampleTimestamps.length;
		for (int i = 0; i < sampleCount; i++) {
			// Read wheel positions and deltas from each module
			SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
			SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
			for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
				modulePositions[moduleIndex] = modules[moduleIndex]
						.getOdometryPositions()[i];
				moduleDeltas[moduleIndex] = new SwerveModulePosition(
						modulePositions[moduleIndex].distanceMeters
								- lastModulePositions[moduleIndex].distanceMeters,
						modulePositions[moduleIndex].angle);
				lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
			}
			// Update gyro angle
			if (gyroInputs.connected) {
				// Use the real gyro angle
				rawGyroRotation = gyroInputs.odometryYawPositions[i];
			} else {
				// Use the angle delta from the kinematics and module deltas
				Twist2d twist = kinematics.toTwist2d(moduleDeltas);
				rawGyroRotation = rawGyroRotation
						.plus(new Rotation2d(twist.dtheta));
			}
			ChassisSpeeds m_ChassisSpeeds = getChassisSpeeds();
			Translation2d linearFieldVelocity = new Translation2d(
					m_ChassisSpeeds.vxMetersPerSecond,
					m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
			fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
					linearFieldVelocity.getY(),
					m_ChassisSpeeds.omegaRadiansPerSecond);
			// Apply update
			poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation,
					modulePositions);
		}
		collisionDetected = collisionDetected();
		isSkidding = calculateSkidding();
		Logger.recordOutput("Skids", isSkidding);
	}

	/**
	 * Calculates the translational vectors of each module, and confirms it is
	 * within .25 m/s of the median vector. If it isn't that module is said to be
	 * "Skidding"
	 * 
	 * @apiNote TEST ON BOT NEEDED
	 * @param m_ChassisSpeeds
	 * @return
	 */
	public boolean[] calculateSkidding() {
		SwerveModuleState[] moduleStates = getModuleStates();
		ChassisSpeeds currentChassisSpeeds = getChassisSpeeds();
		// Step 1: Create a ChassisSpeeds object with solely the rotation component
		ChassisSpeeds rotationOnlySpeeds = new ChassisSpeeds(0.0, 0.0,
				currentChassisSpeeds.omegaRadiansPerSecond + .05);
		double[] xComponentList = new double[4];
		double[] yComponentList = new double[4];
		// Step 2: Convert it into module states with kinematics
		SwerveModuleState[] rotationalStates = kinematics
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
		boolean[] areModulesSkidding = new boolean[4];
		for (int i = 0; i < 4; i++) {
			double deltaX = xComponentList[i];
			double deltaY = yComponentList[i];
			if (Math.abs(deltaX - deltaMedianX) > DriveConstants.SKID_THRESHOLD
					|| Math.abs(
							deltaY - deltaMedianY) > DriveConstants.SKID_THRESHOLD) {
				areModulesSkidding[i] = true;
			} else {
				areModulesSkidding[i] = false;
			}
		}
		return areModulesSkidding;
	}

	/**
	 * Runs the drive at the desired velocity.
	 *
	 * @param speeds Speeds in meters/sec
	 */
	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		// Calculate module setpoints
		ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
		SwerveModuleState[] setpointStates = kinematics
				.toSwerveModuleStates(discreteSpeeds);
		SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates,
				DriveConstants.kMaxSpeedMetersPerSecond);
		// Send setpoints to modules
		SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			// The module returns the optimized state, useful for logging
			optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
		}
		// Log setpoint states
		Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
		Logger.recordOutput("SwerveStates/SetpointsOptimized",
				optimizedSetpointStates);
	}

	/** Stops the drive. */
	public void stopModulesDumb() {
		setChassisSpeeds(new ChassisSpeeds());
	}

	/**
	 * Stops the drive and turns the modules to an X arrangement to resist
	 * movement. The modules will return to their normal orientations the next
	 * time a nonzero velocity is requested.
	 */
	@Override
	public void stopModules() {
		Rotation2d[] headings = new Rotation2d[4];
		for (int i = 0; i < 4; i++) {
			headings[i] = getModuleTranslations()[i].getAngle();
		}
		kinematics.resetHeadings(headings);
		stopModulesDumb();
	}

	/**
	 * Returns a command to run a quasistatic test in the specified direction.
	 */
	@Override
	public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
		return sysId.quasistatic(direction);
	}

	/** Returns a command to run a dynamic test in the specified direction. */
	@Override
	public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
		return sysId.dynamic(direction);
	}

	/**
	 * Returns the module states (turn angles and drive velocities) for all of
	 * the modules.
	 */
	@AutoLogOutput(key = "SwerveStates/Measured")
	private SwerveModuleState[] getModuleStates() {
		SwerveModuleState[] states = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getState();
		}
		return states;
	}

	/**
	 * Returns the module positions (turn angles and drive positions) for all of
	 * the modules.
	 */
	private SwerveModulePosition[] getModulePositions() {
		SwerveModulePosition[] states = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			states[i] = modules[i].getPosition();
		}
		return states;
	}

	/** Returns the current odometry pose. */
	@AutoLogOutput(key = "Odometry/Robot")
	@Override
	public Pose2d getPose() { return poseEstimator.getEstimatedPosition(); }

	/** Returns the current odometry rotation. */
	@Override
	public Rotation2d getRotation2d() { return getPose().getRotation(); }

	/** Resets the current odometry pose. */
	@Override
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
	}

	/**
	 * Adds a vision measurement to the pose estimator.
	 *
	 * @param visionPose The pose of the robot as measured by the vision camera.
	 * @param timestamp  The timestamp of the vision measurement in seconds.
	 */
	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	/** Returns the maximum linear speed in meters per sec. */
	public double getMaxLinearSpeedMetersPerSec() {
		return DriveConstants.kMaxSpeedMetersPerSecond;
	}

	/** Returns the maximum angular speed in radians per sec. */
	public double getMaxAngularSpeedRadPerSec() {
		return MAX_ANGULAR_SPEED;
	}

	/** Returns an array of module translations. */
	public static Translation2d[] getModuleTranslations() {
		return DriveConstants.kModuleTranslations;
	}

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FLDriveTemp", modules[0].getDriveMotorTemp());
		tempMap.put("FLTurnTemp", modules[0].getTurnMotorTemp());
		tempMap.put("FRDriveTemp", modules[1].getDriveMotorTemp());
		tempMap.put("FRTurnTemp", modules[1].getTurnMotorTemp());
		tempMap.put("BLDriveTemp", modules[2].getDriveMotorTemp());
		tempMap.put("BLTurnTemp", modules[2].getTurnMotorTemp());
		tempMap.put("BRDriveTemp", modules[3].getDriveMotorTemp());
		tempMap.put("BRTurnTemp", modules[3].getTurnMotorTemp());
		return tempMap;
	}

	private void registerSelfCheckHardware() {
		super.registerAllHardware(gyroIO.getSelfCheckingHardware());
		super.registerAllHardware(modules[0].getSelfCheckingHardware());
		super.registerAllHardware(modules[1].getSelfCheckingHardware());
		super.registerAllHardware(modules[2].getSelfCheckingHardware());
		super.registerAllHardware(modules[3].getSelfCheckingHardware());
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		for (Module module : modules) {
			List<SelfChecking> moduleHardware = module.getSelfCheckingHardware();
			for (SelfChecking motor : moduleHardware) {
				if (motor.getHardware() instanceof TalonFX) {
					orchestra.add((TalonFX) motor.getHardware());
				}
			}
		}
		return orchestra;
	}
	@Override
	public double getCurrent(){
		return modules[0].getCurrent() + modules[1].getCurrent() + modules[2].getCurrent() + modules[3].getCurrent();
	}
	@Override
	public SystemStatus getTrueSystemStatus() { return getSystemStatus(); }

	@Override
	public Command getRunnableSystemCheckCommand() {
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() {
		return getOrchestraDevices();
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands
				.sequence(run(() -> setChassisSpeeds(new ChassisSpeeds(1.5, 0, 0)))
						.withTimeout(1.0), runOnce(() -> {
							for (int i = 0; i < modules.length; i++) {
								// Retrieve the corresponding REVSwerveModule from the hashmap
								// Get the name of the current ModulePosition
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								if (i == 0)
									name = "Front Left";
								if (i == 1)
									name = "Front Right";
								if (i == 2)
									name = "Back Left";
								if (i == 3)
									name = "Back Right";
								if (Math.abs(moduleState.speedMetersPerSecond) < 1.3
										|| Math.abs(
												moduleState.speedMetersPerSecond) > 1.7) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow for "
													+ name
													+ moduleState.speedMetersPerSecond,
											false, true);
								}
								//angle could be 0, 180, or mod that
								double angle = moduleState.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 0) >= 10
										&& Math.abs(Math.abs(angle) - 180) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ moduleState.angle.getDegrees(),
											false, true);
								}
							}
							System.out.println("COMPLETED X 1.5");
						}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 1.5, 0)))
								.withTimeout(1.0),
						runOnce(() -> {
							System.out.println("STARTING Y CHECK...");
							for (int i = 0; i < modules.length; i++) {
								SwerveModuleState moduleState = modules[i].getState();
								String name = "";
								if (i == 0)
									name = "Front Left";
								if (i == 1)
									name = "Front Right";
								if (i == 2)
									name = "Back Left";
								if (i == 3)
									name = "Back Right";
								if (Math.abs(moduleState.speedMetersPerSecond) < 1.3
										|| Math.abs(
												moduleState.speedMetersPerSecond) > 1.7) {
									addFault(
											"[System Check] Drive motor encoder velocity too slow for "
													+ name
													+ moduleState.speedMetersPerSecond,
											false, true);
								}
								//angle could be 0, 180, or mod that
								double angle = moduleState.angle.getDegrees();
								if (Math.abs(Math.abs(angle) - 90) >= 10
										&& Math.abs(Math.abs(angle) - 270) >= 10) {
									addFault(
											"[System Check] Turn angle off for " + name
													+ moduleState.angle.getDegrees(),
											false, true);
								}
							}
						}),
						run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
								.withTimeout(2.0),
						run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5)))
								.withTimeout(2.0))
				.until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(getModuleStates());
	}

	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; //?
	}

	@Override
	public Twist2d getFieldVelocity() { return fieldVelocity; }

	@Override
	public Command sysIdDynamicTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'sysIdDynamicTurn'");
	}

	@Override
	public Command sysIdQuasistaticTurn(Direction kforwards) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'sysIdQuasistaticTurn'");
	}
	/**
	 * UNTESTED!
	 */
	@Override
	public void zeroHeading() {
		gyroIO.reset();
		debounce = 1;
	}

	@Override
	public boolean isConnected() { return gyroInputs.connected; }

	private boolean collisionDetected() { return gyroInputs.collisionDetected; }

	@Override
	public boolean isCollisionDetected() { return collisionDetected; }
}
