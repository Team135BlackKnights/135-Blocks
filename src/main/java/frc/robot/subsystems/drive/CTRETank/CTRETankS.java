package frc.robot.subsystems.drive.CTRETank;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class CTRETankS implements DrivetrainS {
	private Pigeon2 pigeon = new Pigeon2(30);
	private TalonFX rightLeader = new TalonFX(
			DriveConstants.kFrontRightDrivePort);
	private TalonFX rightFollower = new TalonFX(
			DriveConstants.kBackRightDrivePort);
	private TalonFX leftLeader = new TalonFX(DriveConstants.kFrontLeftDrivePort);
	private TalonFX leftFollower = new TalonFX(
			DriveConstants.kBackLeftDrivePort);
	private TalonFX[] motors = {rightLeader,rightFollower,leftLeader,leftFollower};
	private static final DCMotorSim[] motorSimModels = {
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001)
	};
	private double dtSeconds = 0.02;
	private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
			DriveConstants.kChassisLength);
	private DifferentialDriveWheelPositions wheelPositions;
	private DifferentialDriveWheelSpeeds wheelSpeeds;
	private DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics, getRotation2d(), getLeftMeters(), getRightMeters(), getPose());
	Field2d robotField = new Field2d();
	public Pose2d pose = new Pose2d(0, 0, getRotation2d());
	private final VelocityDutyCycle m_motorRequest = new VelocityDutyCycle(0);
	private final VoltageOut m_voltReq = new VoltageOut(0.0);
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, // Use default ramp rate (1 V/s)
					holdVoltage, // Reduce dynamic step voltage to 4 to prevent brownout
					timeout, // Use default timeout (10 s)
					// Log state with Phoenix SignalLogger class
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism((volts) -> {
				leftLeader.setControl(m_voltReq.withOutput(volts.in(Volts)));
				rightLeader.setControl(m_voltReq.withOutput(volts.in(Volts)));
			}, null, this));

	public CTRETankS() {
		//do main motors
		TalonFXConfigurator leftConfigurator = leftLeader.getConfigurator();
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
		motorConfig.CurrentLimits.StatorCurrentLimit = 1000;
		motorConfig.MotorOutput.Inverted = InvertedValue
				.valueOf((DriveConstants.kFrontLeftDriveReversed) ? 1 : 0);
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		motorConfig.Feedback.SensorToMechanismRatio = DriveConstants.TrainConstants.kDriveEncoderRot2Meter;
		motorConfig.Slot0.kP = .02;
		leftConfigurator.apply(motorConfig);
		//adjust motorConfig for rightSide
		TalonFXConfigurator rightConfig = rightLeader.getConfigurator();
		motorConfig.MotorOutput.Inverted = InvertedValue
				.valueOf((DriveConstants.kFrontRightDriveReversed) ? 1 : 0);
		rightConfig.apply(motorConfig);
		//do followers
		leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
		rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
		poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
				getRotation2d(), getLeftMeters(), getRightMeters(), pose);
		AutoBuilder.configureLTV(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setChassisSpeeds, .02,
				new ReplanningConfig(true, true), () -> Robot.isRed, this);

	}

	private double getLeftMeters() {
		return (leftLeader.getPosition().getValueAsDouble()
				+ leftFollower.getPosition().getValueAsDouble()) / 2;
	}

	private double getRightMeters() {
		return (rightLeader.getPosition().getValueAsDouble()
				+ rightFollower.getPosition().getValueAsDouble()) / 2;
	}

	private double getLeftVelocity() {
		return (leftLeader.getVelocity().getValueAsDouble()
				+ leftFollower.getVelocity().getValueAsDouble()) / 2;
	}

	private double getRightVelocity() {
		return (rightLeader.getVelocity().getValueAsDouble()
				+ rightFollower.getVelocity().getValueAsDouble()) / 2;
	}

	private DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftVelocity(),
				getRightVelocity());
	}

	private DifferentialDriveWheelPositions getWheelPositions() {
		return new DifferentialDriveWheelPositions(getLeftMeters(),
				getRightMeters());
	}

	private double metersPerSecondToRotationsPerSecond(
			double velocityMetersPerSecond) {
		double wheelCircumferenceMeters = Math.PI
				* DriveConstants.TrainConstants.kWheelDiameter;
		return (velocityMetersPerSecond / wheelCircumferenceMeters)
				* DriveConstants.TrainConstants.kDriveMotorGearRatio;
	}

	public Rotation2d LastAngle = new Rotation2d();

	@Override
	public void periodic() {
		wheelPositions = getWheelPositions();
		wheelSpeeds = getWheelSpeeds();
		poseEstimator.update(getRotation2d(), getWheelPositions());
		pose = poseEstimator.getEstimatedPosition();
		if (Constants.currentMode == Constants.Mode.SIM) {
			for (int i = 0; i < motors.length; i++) {
			var motorSim = motors[i].getSimState();
			motorSim.setSupplyVoltage(12);
			motorSimModels[i].setInputVoltage(motorSim.getMotorVoltage());
			motorSimModels[i].update(dtSeconds);
			motorSim.setRawRotorPosition(
					motorSimModels[i].getAngularPositionRotations());
			motorSim.setRotorVelocity(Units.radiansToRotations(
					motorSimModels[i].getAngularVelocityRadPerSec()));}
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
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics
				.toWheelSpeeds(speeds);
		wheelSpeeds.desaturate(DriveConstants.kMaxSpeedMetersPerSecond);
		double leftVelocityRotationsPerSecond = metersPerSecondToRotationsPerSecond(
				wheelSpeeds.leftMetersPerSecond);
		// Set the motor control to the converted velocity
		leftLeader.setControl(
				m_motorRequest.withVelocity(leftVelocityRotationsPerSecond));
		double rightVelocityRotationsPerSecond = metersPerSecondToRotationsPerSecond(
				wheelSpeeds.rightMetersPerSecond);
		// Set the motor control to the converted velocity
		rightLeader.setControl(
				m_motorRequest.withVelocity(rightVelocityRotationsPerSecond));
		if (Constants.currentMode == Constants.Mode.SIM) {
			Pigeon2SimState gyroSim = pigeon.getSimState();
			double angleChange = speeds.omegaRadiansPerSecond * dtSeconds;
			LastAngle = LastAngle.plus(Rotation2d.fromRadians(angleChange));
			gyroSim.setRawYaw(LastAngle.getDegrees());
		}
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(wheelSpeeds);
	}

	@Override
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(getRotation2d(), wheelPositions, pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Pose2d getPose() { return pose; }

	@Override
	public void stopModules() {
		leftLeader.setControl(m_motorRequest.withVelocity(0));
		rightLeader.setControl(m_motorRequest.withVelocity(0));
	}

	@Override
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
	}

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

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		return m_sysIdRoutine.quasistatic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		return m_sysIdRoutine.quasistatic(direction);
	}

	@Override
	public void zeroHeading() { pigeon.reset(); }

	@Override
	public boolean isConnected() {
		return pigeon.getFault_Hardware().getValue();
	}
}
