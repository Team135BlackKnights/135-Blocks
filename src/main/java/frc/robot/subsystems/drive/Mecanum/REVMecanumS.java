package frc.robot.subsystems.drive.Mecanum;

import edu.wpi.first.math.Matrix;
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
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.MotorConstantContainer;

public class REVMecanumS implements DrivetrainS {
	//TODO: mecanum sim, pathPlanner support
	private static CANSparkBase[] sparkMotors = new CANSparkBase[4];
	private static int[] motorIDs;
	private static double[] wheelSimPositions = { 0, 0, 0, 0
	}, wheelSimVelocities = { 0, 0, 0, 0
	};
	private static LinearSystem<N1, N1, N1>[] wheelLinearSystems = new LinearSystem[4];
	private static RelativeEncoder[] wheelRelativeEncoders = new RelativeEncoder[4];
	private static AHRS gyro;
	private static MecanumDriveKinematics driveKinematics;
	private static MecanumDrivePoseEstimator drivePoseEstimator;
	private static MotorConstantContainer[] wheelConstantContainers = new MotorConstantContainer[4];
	private static double maxDriveVelMetersPerSec;
	private static Pose2d pose = new Pose2d(0,0,new Rotation2d(0));
	private static MecanumDriveWheelSpeeds speeds = new MecanumDriveWheelSpeeds(0,0,0,0);
	private static MecanumDriveWheelPositions positions = new MecanumDriveWheelPositions(0,0,0,0);
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
	 * @param gearing                          gearing of the motor
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
		REVMecanumS.maxDriveVelMetersPerSec = maxDriveVelMetersPerSec;
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
			sparkMotors[i].setIdleMode(IdleMode.kBrake);
			sparkMotors[i].enableVoltageCompensation(12);
			sparkMotors[i].setSmartCurrentLimit(i, i);
			sparkMotors[i].clearFaults();
			sparkMotors[i].burnFlash();
			wheelRelativeEncoders[i] = sparkMotors[i].getEncoder();
			wheelRelativeEncoders[i]
					.setPositionConversionFactor(1 / gearing * kWheelRadiusMeters);
			switch (Constants.currentMode) {
			case SIM:
				wheelLinearSystems[i] = LinearSystemId.identifyVelocitySystem(
						wheelConstantContainers[i].getKv(),
						wheelConstantContainers[i].getKa());
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
				getRotation2d(), getWheelPositions(), pose);
	}

	public MecanumDriveWheelSpeeds getWheelSpeeds() {
		switch (Constants.currentMode) {
			case SIM:
				return new MecanumDriveWheelSpeeds(wheelSimVelocities[0],wheelSimVelocities[1],wheelSimVelocities[2],wheelSimVelocities[3]);

		
			default:
					return new MecanumDriveWheelSpeeds(wheelRelativeEncoders[0].getVelocity(),
				wheelRelativeEncoders[1].getVelocity(),
				wheelRelativeEncoders[2].getVelocity(),
				wheelRelativeEncoders[3].getVelocity());				
		}

	}

	public MecanumDriveWheelPositions getWheelPositions() {
		switch (Constants.currentMode) {
		case SIM:
			return new MecanumDriveWheelPositions(wheelSimPositions[0],
					wheelSimPositions[1], wheelSimPositions[2],
					wheelSimPositions[3]);
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

	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		MecanumDriveWheelSpeeds indSpeeds = driveKinematics.toWheelSpeeds(speeds);
		sparkMotors[0]
				.set(indSpeeds.frontLeftMetersPerSecond / maxDriveVelMetersPerSec);
		sparkMotors[1]
				.set(indSpeeds.frontRightMetersPerSecond / maxDriveVelMetersPerSec);
		sparkMotors[2]
				.set(indSpeeds.rearLeftMetersPerSecond / maxDriveVelMetersPerSec);
		sparkMotors[3]
				.set(indSpeeds.rearRightMetersPerSecond / maxDriveVelMetersPerSec);
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
