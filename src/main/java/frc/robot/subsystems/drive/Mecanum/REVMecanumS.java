package frc.robot.subsystems.drive.Mecanum;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DrivetrainS;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.ejml.simple.UnsupportedOperation;
import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.MotorConstantContainer;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;

public class REVMecanumS implements DrivetrainS {
	//TODO: mecanum sim, pathPlanner support
	private static CANSparkBase[] sparkMotors = new CANSparkBase[4];
	private static int[] motorIDs;
	Field2d robotField = new Field2d();
	private double m_simYaw;
	@SuppressWarnings("unchecked")
	private static LinearSystem<N2, N1, N2>[] linearSystems = new LinearSystem[4];
	@SuppressWarnings("unchecked")
	private static KalmanFilter<N2, N1, N2>[] wheelFilters = new KalmanFilter[4];
	@SuppressWarnings("unchecked")
	private static LinearQuadraticRegulator<N2, N1, N2>[] wheelRegulators = new LinearQuadraticRegulator[4];
	@SuppressWarnings("unchecked")
	private static LinearSystemLoop<N2, N1, N2>[] wheelSystemLoops = new LinearSystemLoop[4];
	private static DCMotorSim[] motorSims = new DCMotorSim[4];
	private static RelativeEncoder[] wheelRelativeEncoders = new RelativeEncoder[4];
	private static AHRS gyro;
	private static MecanumDriveKinematics driveKinematics;
	private static MecanumDrivePoseEstimator drivePoseEstimator;
	private static MotorConstantContainer[] wheelConstantContainers = new MotorConstantContainer[4];
	private static double maxDriveVelMetersPerSec = 0;
	private static double[] desiredVelocities = { 0, 0, 0, 0
	};
	private static double[] currentPositions = { 0, 0, 0, 0
	};
	private static Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (int i = 0; i < 4; i++) {
					sparkMotors[i].setVoltage(volts.in(Volts));
				}
			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				sparkMotors[0].setVoltage(-volts.in(Volts));
				sparkMotors[1].setVoltage(-volts.in(Volts));
				sparkMotors[2].setVoltage(volts.in(Volts));
				sparkMotors[3].setVoltage(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));

	/**
	 * Constructs the mecanum drive object, supports both CANSparkMAXs and
	 * CANSparkFlexs
	 * 
	 * @param frontLeftID                      front left motor id
	 * @param frontRightID                     front right motor id
	 * @param backLeftID                       back left motor id
	 * @param backRightID                      back right motor id
	 * @param frontLeftMotorConstantContainer  constants for the front left motor
	 * @param frontRightMotorConstantContainer constants for the front right
	 *                                            motor
	 * @param backLeftMotorConstantContainer   constants for the back right motor
	 * @param backRightMotorConstantContainer  constants for the back left motor
	 * @param gearing                          gearing of the motor (>1 is a reduction)
	 * @param kWheelRadiusMeters               radius of the wheel in meters
	 * @param maxDriveVelMetersPerSec          maximum drive speed in meters per
	 *                                            second
	 */
	public REVMecanumS(int frontLeftID, int frontRightID, int backLeftID,
			int backRightID, int maxAmps,
			MotorConstantContainer frontLeftMotorConstantContainer,
			MotorConstantContainer frontRightMotorConstantContainer,
			MotorConstantContainer backLeftMotorConstantContainer,
			MotorConstantContainer backRightMotorConstantContainer, double gearing,
			double kWheelRadiusMeters, double maxDriveVelMetersPerSec) {
		wheelConstantContainers = new MotorConstantContainer[] {
				frontLeftMotorConstantContainer, frontRightMotorConstantContainer,
				backLeftMotorConstantContainer, backRightMotorConstantContainer
		};
		motorIDs = new int[] { frontLeftID, frontRightID, backLeftID, backRightID
		};
		for (int i = 0; i < 4; i++) {
			switch (DriveConstants.robotMotorController) {
			case NEO_SPARK_MAX:
				sparkMotors[i] = new CANSparkMax(motorIDs[i], MotorType.kBrushless);
				break;
			case VORTEX_SPARK_FLEX:
				sparkMotors[i] = new CANSparkFlex(motorIDs[i],
						MotorType.kBrushless);
				break;
			default:
				throw new UnsupportedOperation("no REV motortype found");
			}
			linearSystems[i] = LinearSystemId.createDCMotorSystem(
					wheelConstantContainers[i].getKv(),
					wheelConstantContainers[i].getKa());
			sparkMotors[i].setIdleMode(IdleMode.kBrake);
			sparkMotors[i].enableVoltageCompensation(12);
			sparkMotors[i].setSmartCurrentLimit(i, i);
			sparkMotors[i].clearFaults();
			sparkMotors[i].burnFlash();
			wheelRelativeEncoders[i] = sparkMotors[i].getEncoder();
			wheelRelativeEncoders[i]
					.setPositionConversionFactor((1 / gearing )* kWheelRadiusMeters * Math.PI);
			wheelRelativeEncoders[i].setVelocityConversionFactor((1 / gearing )* kWheelRadiusMeters * Math.PI);

			wheelFilters[i] = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(),
					linearSystems[i], VecBuilder.fill(3, 3), VecBuilder.fill(.03, 3),
					.02);
			wheelRegulators[i] = new LinearQuadraticRegulator<N2, N1, N2>(
					linearSystems[i], VecBuilder.fill(10, 10), VecBuilder.fill(12),
					.02);
			wheelSystemLoops[i] = new LinearSystemLoop<>(linearSystems[i],
					wheelRegulators[i], wheelFilters[i], 12, .02);
			switch (Constants.currentMode) {
			case SIM:
				switch (DriveConstants.robotMotorController) {
				case NEO_SPARK_MAX:
					motorSims[i] = new DCMotorSim(linearSystems[i],
							DCMotor.getNEO(1), gearing);
					break;
				case VORTEX_SPARK_FLEX:
					motorSims[i] = new DCMotorSim(linearSystems[i],
							DCMotor.getNeoVortex(maxAmps), gearing);
				default:
					break;
				}
			default:
				break;
			}
		}
		gyro = new AHRS(Port.kUSB);
		driveKinematics = new MecanumDriveKinematics(
				DriveConstants.kModuleTranslations[0],
				DriveConstants.kModuleTranslations[1],
				DriveConstants.kModuleTranslations[2],
				DriveConstants.kModuleTranslations[3]);
		drivePoseEstimator = new MecanumDrivePoseEstimator(driveKinematics,
				getRotation2d(), getWheelPositions(), pose);
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						maxDriveVelMetersPerSec, // Max module speed, in m/s
						DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), () -> Robot.isRed, this // Reference to this subsystem to set requirements
		);
	}

	public MecanumDriveWheelSpeeds getWheelSpeeds() {
		switch (Constants.currentMode) {
		case SIM:
			return new MecanumDriveWheelSpeeds(
					motorSims[0].getAngularVelocityRadPerSec(),
					motorSims[1].getAngularVelocityRadPerSec(),
					motorSims[2].getAngularVelocityRadPerSec(),
					motorSims[3].getAngularVelocityRadPerSec());
		default:
			return new MecanumDriveWheelSpeeds(
					wheelRelativeEncoders[0].getVelocity(),
					wheelRelativeEncoders[1].getVelocity(),
					wheelRelativeEncoders[2].getVelocity(),
					wheelRelativeEncoders[3].getVelocity());
		}
	}

	public MecanumDriveWheelPositions getWheelPositions() {
		switch (Constants.currentMode) {
		case SIM:
			return new MecanumDriveWheelPositions(
					motorSims[0].getAngularPositionRad(),
					motorSims[1].getAngularPositionRad(),
					motorSims[2].getAngularPositionRad(),
					motorSims[3].getAngularPositionRad());
		default:
			return new MecanumDriveWheelPositions(
					wheelRelativeEncoders[0].getPosition(),
					wheelRelativeEncoders[1].getPosition(),
					wheelRelativeEncoders[2].getPosition(),
					wheelRelativeEncoders[3].getPosition());
		}
	}

	@Override
	public void periodic() {

		SmartDashboard.putNumber("X", driveKinematics.toChassisSpeeds(getWheelSpeeds()).omegaRadiansPerSecond);
		drivePoseEstimator.update(getRotation2d(), getWheelPositions());
		robotField.setRobotPose(getPose());
		SmartDashboard.putData(robotField);
		for (int i = 0; i < 4; i++) {
			wheelSystemLoops[i].setNextR(VecBuilder.fill(desiredVelocities[i],(
					currentPositions[i] + desiredVelocities[i] * .2)%(2*Math.PI)));
			switch (Constants.currentMode) {
			case REAL:
				wheelSystemLoops[i].correct(
						VecBuilder.fill(wheelRelativeEncoders[i].getPosition(),
								wheelRelativeEncoders[i].getVelocity()));
				wheelSystemLoops[i].predict(.02);
				sparkMotors[i].setVoltage(wheelSystemLoops[i].getU(0));
				break;
			case SIM:
			ChassisSpeeds speeds = driveKinematics.toChassisSpeeds(getWheelSpeeds());
				m_simYaw += speeds.omegaRadiansPerSecond * 0.02;
				int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
				SimDouble angle = new SimDouble(
						SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
				// NavX expects clockwise positive, but sim outputs clockwise negative
				angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_simYaw), 360));
				motorSims[i].update(.02);
				wheelSystemLoops[i].predict(.02);
				motorSims[i].setInputVoltage(wheelSystemLoops[i].getU(0));
			default:
				break;
			}
		}
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		MecanumDriveWheelSpeeds indSpeeds = driveKinematics.toWheelSpeeds(speeds);

		indSpeeds.desaturate(maxDriveVelMetersPerSec);
		System.out.println(indSpeeds.frontLeftMetersPerSecond);
		desiredVelocities[0] = indSpeeds.frontLeftMetersPerSecond;
		desiredVelocities[1] = indSpeeds.frontRightMetersPerSecond;
		desiredVelocities[2] = indSpeeds.rearLeftMetersPerSecond;
		desiredVelocities[3] = indSpeeds.rearRightMetersPerSecond;
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return driveKinematics.toChassisSpeeds(getWheelSpeeds());
	}

	@Override
	public void resetPose(Pose2d pose) {
		drivePoseEstimator.resetPosition(getRotation2d(), getWheelPositions(),
				pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		drivePoseEstimator.addVisionMeasurement(pose, timestamp);
	}

	@Override
	public Pose2d getPose() { return drivePoseEstimator.getEstimatedPosition(); }

	@Override
	public void stopModules() {
		for (int i = 0; i < 4; i++) {
			sparkMotors[i].set(0);
		}
	}

	@Override
	public Rotation2d getRotation2d() { return Rotation2d.fromDegrees(gyro.getAngle()); }

	/**
	 * @exception THIS CANNOT BE DONE USING MECANUM!
	 */
	@Override
	public Command sysIdDynamicTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Impossible method 'sysIdDynamicTurn' for DriveTrain mecanum");
	}

	/**
	 * @exception THIS CANNOT BE DONE USING MECANUM!
	 */
	@Override
	public Command sysIdQuasistaticTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Impossible method 'sysIdQuasistaticTurn' for DriveTrain mecanum");
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
	public void zeroHeading() { gyro.reset(); }

	@Override
	public boolean isConnected() { return gyro.isConnected(); }
}
