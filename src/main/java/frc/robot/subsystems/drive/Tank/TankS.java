package frc.robot.subsystems.drive.Tank;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
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
import java.util.function.Supplier;

public class TankS implements DrivetrainS {
	private Supplier<Pose2d> pose2dSupplier = () -> {
		return getPose();
	};
	private Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> {
		return getChassisSpeeds();
	};
	private Pose2d pose = new Pose2d();
	private static AHRS gyro = new AHRS();
	private static CANSparkMax leftMaster;
	private static CANSparkMax leftFollower;
	private static CANSparkMax rightMaster;
	private static CANSparkMax rightFollower;
	private static RelativeEncoder leftMasterEncoder;
	private static RelativeEncoder leftFollowerEncoder;
	private static RelativeEncoder rightMasterEncoder;
	private static RelativeEncoder rightFollowerEncoder;
	private DifferentialDrivePoseEstimator poseEstimator;
	private DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(
			DriveConstants.kChassisLength);
	private DifferentialDriveWheelPositions wheelPositions;
	private DifferentialDriveWheelSpeeds wheelSpeeds;
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				leftMaster.setVoltage(volts.in(Volts));
				rightMaster.setVoltage(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				leftMaster.setVoltage(-volts.in(Volts));
				rightMaster.setVoltage(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));
	//Divide the kVLinear by wheelspeed!
	private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId
			.identifyDrivetrainSystem(2.1, .1, 95, 1); //Placeholders.
	private DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
			m_drivetrainSystem, DCMotor.getNEO(4),7.5,DriveConstants.kChassisWidth,DriveConstants.TrainConstants.kWheelDiameter/2,null);
	 double m_currentAngle = 0;
	 double m_simLeftDriveEncoderPosition = 0;
	 double m_simLeftDriveEncoderVelocity = 0;
	 double m_simRightDriveEncoderPosition = 0;
	 double m_simRightDriveEncoderVelocity = 0;
	 double m_simAngleDifference = 0;
	 double m_simTurnAngleIncrement = 0;
	Field2d robotField = new Field2d();
	public TankS(int leftMasterID, int leftFollowerID, int rightMasterID,
			int rightFollowerID, boolean leftMasterInverted,
			boolean leftFollowerInverted, boolean rightMasterInverted,
			boolean rightFollowerInverted, IdleMode idleMode, int maxAmps,
			double gearing, double kWheelRadiusMeters) {
		leftMaster = new CANSparkMax(leftMasterID, MotorType.kBrushless);
		leftFollower = new CANSparkMax(leftFollowerID, MotorType.kBrushless);
		rightMaster = new CANSparkMax(rightMasterID, MotorType.kBrushless);
		rightFollower = new CANSparkMax(rightFollowerID, MotorType.kBrushless);
		leftFollower.follow(leftMaster);
		rightFollower.follow(rightMaster);
		leftMasterEncoder = leftMaster.getEncoder();
		leftFollowerEncoder = leftFollower.getEncoder();
		rightMasterEncoder = rightMaster.getEncoder();
		rightFollowerEncoder = rightFollower.getEncoder();
		leftMasterEncoder
				.setPositionConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		leftMasterEncoder
				.setVelocityConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		leftFollowerEncoder
				.setPositionConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		leftFollowerEncoder
				.setVelocityConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		rightMasterEncoder
				.setPositionConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		rightMasterEncoder
				.setVelocityConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		rightFollowerEncoder
				.setPositionConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		rightFollowerEncoder
				.setVelocityConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
		leftMaster.setInverted(leftMasterInverted);
		leftFollower.setInverted(leftFollowerInverted);
		rightMaster.setInverted(rightMasterInverted);
		rightFollower.setInverted(rightFollowerInverted);
		CANSparkMax[] motors = { leftMaster, leftFollower, rightMaster,
				rightFollower
		};
		for (CANSparkMax motor : motors) {
			motor.setIdleMode(idleMode);
			motor.enableVoltageCompensation(12);
			motor.setSmartCurrentLimit(maxAmps, maxAmps);
			motor.clearFaults();
			motor.burnFlash();
		}
		SmartDashboard.putNumber("ROBOT HEADING TANK",
				getRotation2d().getRadians());
		poseEstimator = new DifferentialDrivePoseEstimator(
				differentialDriveKinematics, getRotation2d(), getLeftMeters(),
				getRightMeters(), pose);
		AutoBuilder.configureLTV(pose2dSupplier, this::resetPose,
				chassisSpeedsSupplier, this::setChassisSpeeds, .02,
				new ReplanningConfig(true, true), () -> Robot.isRed, this);
	}

	public Rotation2d getRotation2d() { 
		if (Constants.currentMode == Constants.Mode.SIM){
			return new Rotation2d(m_currentAngle);
		}
		return gyro.getRotation2d(); 
	}

	private double getLeftMeters() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simLeftDriveEncoderPosition;
		}
		return (leftMasterEncoder.getPosition()
				+ leftFollowerEncoder.getPosition()) / 2;
	}

	private double getRightMeters() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simRightDriveEncoderPosition;
		}
		return (rightMasterEncoder.getPosition()
				+ rightFollowerEncoder.getPosition()) / 2;
	}

	private double getLeftVelocity() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simLeftDriveEncoderVelocity;
		}
		return (leftMasterEncoder.getVelocity()
				+ leftFollowerEncoder.getVelocity()) / 2;
	}

	private double getRightVelocity() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simRightDriveEncoderVelocity;
		}
		return (rightMasterEncoder.getVelocity()
				+ rightFollowerEncoder.getVelocity()) / 2;
	}

	/**
	 * Speeds CANNOT have a Y (argument 2 MUST be zero)
	 */
	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		DifferentialDriveWheelSpeeds wheelSpeeds = differentialDriveKinematics
				.toWheelSpeeds(speeds);
		wheelSpeeds.desaturate(DriveConstants.kMaxSpeedMetersPerSecond);
		double leftVelocity = wheelSpeeds.leftMetersPerSecond
				/ DriveConstants.kMaxSpeedMetersPerSecond;
		double rightVelocity = wheelSpeeds.rightMetersPerSecond
				/ DriveConstants.kMaxSpeedMetersPerSecond;
		leftMaster.set(leftVelocity);
		rightMaster.set(rightVelocity);
		if (Constants.currentMode == Constants.Mode.SIM) {
			simUpdateDrivePosition(wheelSpeeds);
		}
	}

	private void simUpdateDrivePosition(DifferentialDriveWheelSpeeds speeds) {
		m_simLeftDriveEncoderVelocity = speeds.leftMetersPerSecond;
		double distancePer20MsL = m_simLeftDriveEncoderVelocity * .02;
		m_simLeftDriveEncoderVelocity += distancePer20MsL;
		m_simRightDriveEncoderVelocity = speeds.rightMetersPerSecond;
		double distancePer20MsR = m_simRightDriveEncoderVelocity * .02;
		m_simRightDriveEncoderVelocity += distancePer20MsR;
	}

	@Override
	public void periodic() {
		wheelPositions = getWheelPositions();
		wheelSpeeds = getWheelSpeeds();
		pose = poseEstimator.update(getRotation2d(), getWheelPositions());
		if (Constants.currentMode == Constants.Mode.SIM) {
			drivetrainSim.setInputs(
					leftMaster.get() * 12,
					rightMaster.get() * 12);
			drivetrainSim.update(0.02);
			m_simLeftDriveEncoderPosition = drivetrainSim.getLeftPositionMeters();
			m_simLeftDriveEncoderVelocity = drivetrainSim.getLeftVelocityMetersPerSecond();
			m_simRightDriveEncoderPosition = drivetrainSim.getRightPositionMeters();
			m_simRightDriveEncoderVelocity = drivetrainSim.getRightVelocityMetersPerSecond();
			m_currentAngle = drivetrainSim.getHeading().getDegrees();
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

	private DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftVelocity(),
				getRightVelocity());
	}

	private DifferentialDriveWheelPositions getWheelPositions() {
		return new DifferentialDriveWheelPositions(getLeftMeters(),
				getRightMeters());
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return differentialDriveKinematics.toChassisSpeeds(wheelSpeeds);
	}

	@Override
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(getRotation2d(), wheelPositions, pose);
		drivetrainSim.setPose(pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Pose2d getPose() { return pose; }

	@Override
	public void stopModules() { setChassisSpeeds(new ChassisSpeeds(0, 0, 0)); }

	/**
	 * @exception THIS CANNOT BE DONE USING TANK!
	 */
	@Override
	public Command sysIdDynamicTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Impossible method 'sysIdDynamicTurn' for DriveTrain tank");
	}

	/**
	 * @exception THIS CANNOT BE DONE USING TANK!
	 */
	@Override
	public Command sysIdQuasistaticTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Impossible method 'sysIdQuasistaticTurn' for DriveTrain tank");
	}

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	@Override
	public void zeroHeading() {
		gyro.reset();
		poseEstimator.resetPosition(getRotation2d(), wheelPositions, pose);
	}
	@Override
	public boolean isConnected(){
		return gyro.isConnected();
	}
}
