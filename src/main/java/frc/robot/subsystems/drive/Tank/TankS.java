package frc.robot.subsystems.drive.Tank;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
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
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
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
	private DifferentialDrivePoseEstimator poseEstimator;
	private static CANSparkBase[] motors = new CANSparkBase[4];
	private static RelativeEncoder[] encoders = new RelativeEncoder[4];

	private DifferentialDriveKinematics differentialDriveKinematics = new DifferentialDriveKinematics(
			DriveConstants.kChassisLength);
	private DifferentialDriveWheelPositions wheelPositions;
	private DifferentialDriveWheelSpeeds wheelSpeeds;
	private static final DCMotorSim[] motorSimModels = {
			new DCMotorSim(DCMotor.getNEO(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getNEO(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getNEO(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getNEO(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001)
	};
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				motors[0].setVoltage(volts.in(Volts));
				motors[2].setVoltage(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				motors[0].setVoltage(-volts.in(Volts));
				motors[2].setVoltage(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));
	//Divide the kVLinear by wheelspeed!
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
		int[] motorIDs = {leftMasterID, leftFollowerID, rightMasterID, rightFollowerID};
		boolean[] motorsInverted = {leftMasterInverted, leftFollowerInverted, rightMasterInverted, rightFollowerInverted};
		for (int i = 0; i <4; i++){
			switch (DriveConstants.robotMotorController) {
				case NEO_SPARK_MAX:
				motors[i] = new CANSparkMax(motorIDs[i], MotorType.kBrushless);
					break;
				case VORTEX_SPARK_FLEX:
				motors[i] = new CANSparkFlex(motorIDs[i], MotorType.kBrushless);
				break;
				default:
					break;
			}
			encoders[i] = motors[i].getEncoder();
			encoders[i].setPositionConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
			encoders[i].setVelocityConversionFactor(DriveConstants.TrainConstants.kDriveEncoderRot2Meter);
			motors[i].setInverted(motorsInverted[i]);
			if (i%2 == 0){
				motors[i+1].follow(motors[i]);
			}
			motors[i].setIdleMode(idleMode);
			motors[i].enableVoltageCompensation(12);
			motors[i].setSmartCurrentLimit(maxAmps, maxAmps);
			motors[i].clearFaults();
			motors[i].burnFlash();
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
		return (encoders[0].getPosition()
				+ encoders[1].getPosition()) / 2;
	}

	private double getRightMeters() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simRightDriveEncoderPosition;
		}
		return (encoders[2].getPosition()
				+ encoders[3].getPosition()) / 2;
	}

	private double getLeftVelocity() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simLeftDriveEncoderVelocity;
		}
		return (encoders[0].getVelocity()
				+ encoders[1].getVelocity()) / 2;
	}

	private double getRightVelocity() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return m_simRightDriveEncoderVelocity;
		}
		return (encoders[2].getVelocity()
				+ encoders[3].getVelocity()) / 2;
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
		motors[0].set(leftVelocity);
		motors[2].set(rightVelocity);
	}


	@Override
	public void periodic() {
		wheelPositions = getWheelPositions();
		wheelSpeeds = getWheelSpeeds();
		pose = poseEstimator.update(getRotation2d(), getWheelPositions());
		if (Constants.currentMode == Constants.Mode.SIM) {
			for (int i = 0; i < motors.length; i++){

			}
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
		//drivetrainSim.setPose(pose);
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
