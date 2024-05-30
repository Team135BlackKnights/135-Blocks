package frc.robot.subsystems.drive.Mecanum;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
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
import org.ejml.simple.UnsupportedOperation;
import org.littletonrobotics.junction.Logger;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.MotorConstantContainer;
import edu.wpi.first.math.estimator.KalmanFilter;

public class REVMecanumS implements DrivetrainS {
	//TODO: mecanum sim
	private static CANSparkFlex[] sparkFlexes = new CANSparkFlex[4];
	private static CANSparkMax[] sparkMaxes = new CANSparkMax[4];
	private static int[] motorIDs;
	private static LinearSystem<N1, N1, N1>[] wheelLinearSystems = new LinearSystem[4];
	private static KalmanFilter<N1, N1, N1>[] kalmanFilters = new KalmanFilter[4];
	private static RelativeEncoder[] wheelRelativeEncoders = new RelativeEncoder[4];
	private static LinearQuadraticRegulator<N1, N1, N1>[] linearQuadraticRegulators = new LinearQuadraticRegulator[4];
	private static AHRS gyro;
	private static MecanumDriveKinematics driveKinematics;
	private static MecanumDrivePoseEstimator drivePoseEstimator;
	private static MotorConstantContainer[] wheelConstantContainers = new MotorConstantContainer[4];
	private static LinearSystemLoop[] systemLoops = new LinearSystemLoop[4];
	private static LinearPlantInversionFeedforward[] linearPlantInversionFeedforwards = new LinearPlantInversionFeedforward[4];
	private static Pose2d pose;
	private static MecanumDriveWheelPositions wheelPositions;
	private static MecanumDriveWheelSpeeds speeds;
	private static double maxDriveVelMetersPerSec;
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (int i = 0; i < 4; i++) {
				switch (DriveConstants.robotMotorController) {
					case NEO_SPARK_MAX:
					sparkMaxes[i].setVoltage(volts.in(Volts));
	
					break;
					case VORTEX_SPARK_FLEX:
					sparkFlexes[i].setVoltage(volts.in(Volts));
					break;
				}}

			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				switch (DriveConstants.robotMotorController) {
					case NEO_SPARK_MAX:
					sparkMaxes[0].setVoltage(-volts.in(Volts));
					sparkMaxes[1].setVoltage(-volts.in(Volts));
					sparkMaxes[2].setVoltage(volts.in(Volts));
					sparkMaxes[3].setVoltage(volts.in(Volts));
					break;
					case VORTEX_SPARK_FLEX:
					sparkFlexes[0].setVoltage(-volts.in(Volts));
					sparkFlexes[1].setVoltage(-volts.in(Volts));
					sparkFlexes[2].setVoltage(volts.in(Volts));
					sparkFlexes[3].setVoltage(volts.in(Volts));
					break;
				}

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
	 * @param gearing gearing of the motor
	 * @param kWheelRadiusMeters radius of the wheel in meters
	 * @param maxDriveVelMetersPerSec maximum drive speed in meters per second
	 */
	public REVMecanumS(int frontLeftID, int frontRightID, int backLeftID,
			int backRightID, int maxAmps,
			MotorConstantContainer frontLeftMotorConstantContainer,
			MotorConstantContainer frontRightMotorConstantContainer,
			MotorConstantContainer backLeftMotorConstantContainer,
			MotorConstantContainer backRightMotorConstantContainer,
			double gearing,
			double kWheelRadiusMeters, double maxDriveVelMetersPerSec) {
			REVMecanumS.maxDriveVelMetersPerSec = maxDriveVelMetersPerSec;
			pose = new Pose2d();
			wheelPositions = new MecanumDriveWheelPositions();
			speeds = new MecanumDriveWheelSpeeds();
		wheelConstantContainers = new MotorConstantContainer[] {
				frontLeftMotorConstantContainer, frontRightMotorConstantContainer,
				backLeftMotorConstantContainer, backRightMotorConstantContainer
		};
		motorIDs = new int[] { frontLeftID, frontRightID, backLeftID, backRightID
		};
		for (int i = 0; i < 4; i++) {
			switch (DriveConstants.robotMotorController) {
			case NEO_SPARK_MAX:
				sparkMaxes[i] = new CANSparkMax(motorIDs[i], MotorType.kBrushless);
				sparkMaxes[i].setIdleMode(IdleMode.kBrake);
				sparkMaxes[i].enableVoltageCompensation(12);
				sparkMaxes[i].setSmartCurrentLimit(i, i);
				sparkMaxes[i].clearFaults();
				sparkMaxes[i].burnFlash();
				wheelRelativeEncoders[i] = sparkMaxes[i].getEncoder();
				break;
			case VORTEX_SPARK_FLEX:
				sparkFlexes[i] = new CANSparkFlex(motorIDs[i],
						MotorType.kBrushless);
				sparkFlexes[i].setIdleMode(IdleMode.kBrake);
				sparkFlexes[i].enableVoltageCompensation(12);
				sparkFlexes[i].setSmartCurrentLimit(i, i);
				sparkFlexes[i].clearFaults();
				sparkFlexes[i].burnFlash();
				wheelRelativeEncoders[i] = sparkMaxes[i].getEncoder();
				break;
			default:
				throw new UnsupportedOperation(
						"dawg you have to use a REV motortype");
						
			}
			wheelRelativeEncoders[i].setPositionConversionFactor(1 / gearing * kWheelRadiusMeters);
			switch (Constants.currentMode) {
			case SIM:
				wheelLinearSystems[i] = LinearSystemId.identifyVelocitySystem(
						wheelConstantContainers[i].getKv(),
						wheelConstantContainers[i].getKa());
				kalmanFilters[i] = new KalmanFilter<N1, N1, N1>(Nat.N1(), Nat.N1(),
						wheelLinearSystems[i], VecBuilder.fill(3),
						VecBuilder.fill(.01), .02);
				linearQuadraticRegulators[i] = new LinearQuadraticRegulator<>(
						wheelLinearSystems[i], VecBuilder.fill(8.0),
						VecBuilder.fill(12.0), .02);
				linearPlantInversionFeedforwards[i] = new LinearPlantInversionFeedforward<N1, N1, N1>(
						wheelLinearSystems[i], .02);
				systemLoops[i] = new LinearSystemLoop<N1, N1, N1>(
						linearQuadraticRegulators[i],
						linearPlantInversionFeedforwards[i], kalmanFilters[i], 12);
				break;
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
				getRotation2d(), getWheelPositions(), getPose());
	}

	public MecanumDriveWheelSpeeds getWheelSpeeds() {
		return new MecanumDriveWheelSpeeds(wheelRelativeEncoders[0].getVelocity(),
				wheelRelativeEncoders[1].getVelocity(),
				wheelRelativeEncoders[2].getVelocity(),
				wheelRelativeEncoders[3].getVelocity());
	}

	public MecanumDriveWheelPositions getWheelPositions() {
		return new MecanumDriveWheelPositions(
				wheelRelativeEncoders[0].getPosition(),
				wheelRelativeEncoders[1].getPosition(),
				wheelRelativeEncoders[2].getPosition(),
				wheelRelativeEncoders[3].getPosition());
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		MecanumDriveWheelSpeeds indSpeeds = driveKinematics.toWheelSpeeds(speeds);
		switch (DriveConstants.robotMotorController) {
			case NEO_SPARK_MAX:
				sparkMaxes[0].set(indSpeeds.frontLeftMetersPerSecond/maxDriveVelMetersPerSec);
				sparkMaxes[1].set(indSpeeds.frontRightMetersPerSecond/maxDriveVelMetersPerSec);
				sparkMaxes[2].set(indSpeeds.rearLeftMetersPerSecond/maxDriveVelMetersPerSec);
				sparkMaxes[3].set(indSpeeds.rearRightMetersPerSecond/maxDriveVelMetersPerSec);
				break;
			case VORTEX_SPARK_FLEX:
				sparkFlexes[0].set(indSpeeds.frontLeftMetersPerSecond/maxDriveVelMetersPerSec);
				sparkFlexes[1].set(indSpeeds.frontRightMetersPerSecond/maxDriveVelMetersPerSec);
				sparkFlexes[2].set(indSpeeds.rearLeftMetersPerSecond/maxDriveVelMetersPerSec);
				sparkFlexes[3].set(indSpeeds.rearRightMetersPerSecond/maxDriveVelMetersPerSec);
				break;
			default:
				throw new UnsupportedOperation(
						"dawg you have to use a REV motortype");
						
			}
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
			switch (DriveConstants.robotMotorController) {
			case NEO_SPARK_MAX:
				sparkMaxes[i].set(0);
				break;
			case VORTEX_SPARK_FLEX:
				sparkFlexes[i].set(0);
				break;
			default:
				throw new UnsupportedOperation(
						"dawg you have to use a REV motortype");
			}
		}
	}

	@Override
	public Rotation2d getRotation2d() { return new Rotation2d(gyro.getAngle()); }
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
