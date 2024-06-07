package frc.robot.subsystems.drive.REVTank;

import org.littletonrobotics.junction.Logger;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
import frc.robot.utils.drive.Position;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class REVTankS implements DrivetrainS {
	private Pose2d pose = new Pose2d();
	private static AHRS gyro = new AHRS();
	private DifferentialDrivePoseEstimator poseEstimator;
	private static CANSparkBase[] motors = new CANSparkBase[4];
	private Twist2d fieldVelocity = new Twist2d();
	private static RelativeEncoder[] encoders = new RelativeEncoder[4];
	private static final DCMotorSim[] motorSimModels = new DCMotorSim[4];
	private DifferentialDriveKinematics differentialDriveKinematics;
	private Position<DifferentialDriveWheelPositions> wheelPositions;
	private DifferentialDriveWheelSpeeds wheelSpeeds;
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
	 double dtSeconds = 0.02;
	 double kDriveEncoderRot2Meter, maxSpeed;
	Field2d robotField = new Field2d();
	public REVTankS(REVTankConstantContainer container) {
			differentialDriveKinematics	 = new DifferentialDriveKinematics(
			container.getChassisLength());
		int[] motorIDs = {container.getLeftMasterID(), container.getLeftFollowerID(), container.getRightMasterID(), container.getRightFollowerID()};
		boolean[] motorsInverted = {container.getLeftMasterInverted(), container.getLeftFollowerInverted(), container.getRightMasterInverted(), container.getRightFollowerInverted()};
		for (int i = 0; i <4; i++){
			switch (DriveConstants.robotMotorController) {
				case NEO_SPARK_MAX:
				motors[i] = new CANSparkMax(motorIDs[i], MotorType.kBrushless);
				motorSimModels[i] = new DCMotorSim(DCMotor.getNEO(1), container.getGearing(), .001);
				this.maxSpeed = (5676 / 60.0) / container.getGearing() * (Math.PI * container.getWheelDiameterMeters());
					break;
				case VORTEX_SPARK_FLEX:
				motors[i] = new CANSparkFlex(motorIDs[i], MotorType.kBrushless);
				motorSimModels[i] = new DCMotorSim(DCMotor.getNeoVortex(1), container.getGearing(), .001);
				this.maxSpeed = (6784 / 60.0) / container.getGearing() * (Math.PI * container.getWheelDiameterMeters());

				break;
				default:
					break;
			}
			 kDriveEncoderRot2Meter = container.getGearing() * Math.PI* container.getWheelDiameterMeters();
			encoders[i] = motors[i].getEncoder();
			encoders[i].setPositionConversionFactor(kDriveEncoderRot2Meter);
			encoders[i].setVelocityConversionFactor(kDriveEncoderRot2Meter);
			motors[i].setInverted(motorsInverted[i]);
			if (i%2 == 1){
				motors[i].follow(motors[i-1]);
			}
			motors[i].setIdleMode(container.getIdleMode());
			motors[i].enableVoltageCompensation(12);
			motors[i].setSmartCurrentLimit(container.getMaxAmps(), container.getMaxAmps());
			motors[i].clearFaults();
			motors[i].burnFlash();
		}

		SmartDashboard.putNumber("ROBOT HEADING TANK",
				getRotation2d().getRadians());
		poseEstimator = new DifferentialDrivePoseEstimator(
				differentialDriveKinematics, getRotation2d(), getLeftMeters(),
				getRightMeters(), pose);
		AutoBuilder.configureLTV(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setChassisSpeeds, .02,
				new ReplanningConfig(true, true), () -> Robot.isRed, this);
	}

	public Rotation2d getRotation2d() { 
		if (Constants.currentMode == Constants.Mode.SIM){
			return new Rotation2d(m_currentAngle);
		}
		return gyro.getRotation2d(); 
	}

	private double getLeftMeters() {
		return (encoders[0].getPosition()
				+ encoders[1].getPosition()) / 2;
	}

	private double getRightMeters() {
		return (encoders[2].getPosition()
				+ encoders[3].getPosition()) / 2;
	}

	private double getLeftVelocity() {
		return (encoders[0].getVelocity()
				+ encoders[1].getVelocity()) / 2;
	}

	private double getRightVelocity() {
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
		wheelSpeeds.desaturate(maxSpeed);
		double leftVelocity = wheelSpeeds.leftMetersPerSecond
				/ maxSpeed;
		double rightVelocity = wheelSpeeds.rightMetersPerSecond
				/ maxSpeed;
		motors[0].set(leftVelocity);
		motors[2].set(rightVelocity);
		m_currentAngle += speeds.omegaRadiansPerSecond * 0.02;
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(
					SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// NavX expects clockwise positive, but sim outputs clockwise negative
			angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_currentAngle), 360));
			//m_pigeon.getSimCollection().setRawHeading(-Units.radiansToDegrees(m_simYaw));
	}
	@Override
	public void periodic() {
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		wheelSpeeds = getWheelSpeeds();
		pose = poseEstimator.update(getRotation2d(), getWheelPositions());
		if (Constants.currentMode == Constants.Mode.SIM) {
			for (int i=0; i < 4; i+=2){
				motorSimModels[i].setInputVoltage(motors[i].get()*12);
				motorSimModels[i].update(dtSeconds);
				encoders[i].setPosition(motorSimModels[i].getAngularPositionRotations());
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
		ChassisSpeeds m_ChassisSpeeds = differentialDriveKinematics.toChassisSpeeds(wheelSpeeds);
		Translation2d linearFieldVelocity = new Translation2d(
			m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omegaRadiansPerSecond);
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
		poseEstimator.resetPosition(getRotation2d(), wheelPositions.getPositions(), pose);
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
		poseEstimator.resetPosition(getRotation2d(), wheelPositions.getPositions(), pose);
	}
	@Override
	public boolean isConnected(){
		return gyro.isConnected();
	}
	@Override
	public double getYawVelocity() {
		return fieldVelocity.dtheta; //?
	}

	@Override
	public Twist2d getFieldVelocity() {
		return fieldVelocity;
	 }
	
}
