// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingPigeon2;
import frc.robot.utils.selfCheck.SelfCheckingSparkBase;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of
 * "CANSparkMax" with "CANSparkFlex".
 */
public class MecanumIOSparkBase implements MecanumIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private final CANSparkBase frontLeft;
	private final CANSparkBase frontRight;
	private final CANSparkBase backLeft;
	private final CANSparkBase backRight;
	private final RelativeEncoder frontLeftEncoder;
	private final RelativeEncoder frontRightEncoder;
	private final RelativeEncoder backLeftEncoder;
	private final RelativeEncoder backRightEncoder;
	private final SparkPIDController frontLeftPID;
	private final SparkPIDController frontRightPID;
	private final SparkPIDController backLeftPID;
	private final SparkPIDController backRightPID;
	private final Pigeon2 pigeon = new Pigeon2(30);
	private final StatusSignal<Double> yaw = pigeon.getYaw();
	private final StatusSignal<Double> accelX = pigeon.getAccelerationX();
	private final StatusSignal<Double> accelY = pigeon.getAccelerationY();
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;

	public MecanumIOSparkBase() {
		if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
			frontLeft = new CANSparkMax(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			frontRight = new CANSparkMax(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			backLeft = new CANSparkMax(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			backRight = new CANSparkMax(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
		} else {
			frontLeft = new CANSparkFlex(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			frontRight = new CANSparkFlex(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			backLeft = new CANSparkFlex(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			backRight = new CANSparkFlex(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
		}
		frontLeftEncoder = frontLeft.getEncoder();
		frontRightEncoder = frontRight.getEncoder();
		backLeftEncoder = backLeft.getEncoder();
		backRightEncoder = backRight.getEncoder();
		frontLeftPID = frontLeft.getPIDController();
		frontRightPID = frontRight.getPIDController();
		backLeftPID = backLeft.getPIDController();
		backRightPID = backRight.getPIDController();
		frontLeft.restoreFactoryDefaults();
		frontRight.restoreFactoryDefaults();
		backLeft.restoreFactoryDefaults();
		backRight.restoreFactoryDefaults();
		frontLeft.setCANTimeout(250);
		frontRight.setCANTimeout(250);
		backLeft.setCANTimeout(250);
		backRight.setCANTimeout(250);
		frontLeft.setInverted(DriveConstants.kFrontLeftDriveReversed);
		frontRight.setInverted(DriveConstants.kFrontRightDriveReversed);
		frontLeft.enableVoltageCompensation(12.0);
		frontRight.enableVoltageCompensation(12.0);
		frontLeft.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		frontRight.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		frontLeftPID.setP(KP);
		frontLeftPID.setD(KD);
		frontRightPID.setP(KP);
		frontRightPID.setD(KD);
		backLeftPID.setP(KP);
		backLeftPID.setD(KD);
		backRightPID.setP(KP);
		backRightPID.setD(KD);
		frontLeft.burnFlash();
		frontRight.burnFlash();
		backLeft.burnFlash();
		backRight.burnFlash();
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(100.0);
		pigeon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(MecanumIOInputs inputs) {
		inputs.leftFrontPositionRad = Units
				.rotationsToRadians(frontLeftEncoder.getPosition() / GEAR_RATIO);
		inputs.leftFrontVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						frontLeftEncoder.getVelocity() / GEAR_RATIO);
		inputs.leftFrontAppliedVolts = frontLeft.getAppliedOutput()
				* frontLeft.getBusVoltage();
		inputs.leftBackPositionRad = Units
				.rotationsToRadians(backLeftEncoder.getPosition() / GEAR_RATIO);
		inputs.leftBackVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						backLeftEncoder.getVelocity() / GEAR_RATIO);
		inputs.leftBackAppliedVolts = backLeft.getAppliedOutput()
				* backLeft.getBusVoltage();
		inputs.leftCurrentAmps = new double[] { frontLeft.getOutputCurrent(),
				backLeft.getOutputCurrent()
		};
		inputs.frontLeftDriveTemp = frontLeft.getMotorTemperature();
		inputs.backLeftDriveTemp = backLeft.getMotorTemperature();
		inputs.rightFrontPositionRad = Units
				.rotationsToRadians(frontRightEncoder.getPosition() / GEAR_RATIO);
		inputs.rightFrontVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						frontRightEncoder.getVelocity() / GEAR_RATIO);
		inputs.rightFrontAppliedVolts = frontRight.getAppliedOutput()
				* frontRight.getBusVoltage();
		inputs.rightBackPositionRad = Units
				.rotationsToRadians(backRightEncoder.getPosition() / GEAR_RATIO);
		inputs.rightBackVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						backRightEncoder.getVelocity() / GEAR_RATIO);
		inputs.rightBackAppliedVolts = backRight.getAppliedOutput()
				* backRight.getBusVoltage();
		inputs.rightCurrentAmps = new double[] { frontRight.getOutputCurrent(),
				backRight.getOutputCurrent()
		};
		inputs.frontRightDriveTemp = frontRight.getMotorTemperature();
		inputs.backRightDriveTemp = backRight.getMotorTemperature();
		inputs.gyroConnected = BaseStatusSignal.refreshAll(yaw, accelX, accelY)
				.equals(StatusCode.OK);
		inputs.gyroYaw = Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble());
		double curr_world_linear_accel_x = accelX.getValueAsDouble();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = accelY.getValueAsDouble();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G) //if we suddenly move .5 G's
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			inputs.collisionDetected = true;
		}
		inputs.collisionDetected = false;
	}

	@Override
	public void setVoltage(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {
		frontLeft.setVoltage(frontLeftVolts);
		frontRight.setVoltage(frontRightVolts);
		backLeft.setVoltage(backLeftVolts);
		backRight.setVoltage(backRightVolts);
	}

	@Override
	public void reset() { pigeon.reset(); }

	@Override
	public void setVelocity(double frontLeftRadPerSec,
			double frontRightRadPerSec, double backLeftRadPerSec,
			double backRightRadPerSec, double frontLeftFFVolts,
			double frontRightFFVolts, double backLeftFFVolts,
			double backRightFFVolts) {
		frontLeftPID.setReference(
				Units.radiansPerSecondToRotationsPerMinute(
						frontLeftRadPerSec * GEAR_RATIO),
				ControlType.kVelocity, 0, frontLeftFFVolts);
		frontRightPID.setReference(
				Units.radiansPerSecondToRotationsPerMinute(
						frontRightRadPerSec * GEAR_RATIO),
				ControlType.kVelocity, 0, frontRightFFVolts);
		backLeftPID.setReference(
				Units.radiansPerSecondToRotationsPerMinute(
						backLeftRadPerSec * GEAR_RATIO),
				ControlType.kVelocity, 0, backLeftFFVolts);
		backRightPID.setReference(
				Units.radiansPerSecondToRotationsPerMinute(
						backRightRadPerSec * GEAR_RATIO),
				ControlType.kVelocity, 0, backRightFFVolts);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingPigeon2("IMU", pigeon));
		hardware.add(new SelfCheckingSparkBase("FrontLeftDrive", frontLeft));
		hardware.add(new SelfCheckingSparkBase("BackLeftDrive", backLeft));
		hardware.add(new SelfCheckingSparkBase("FrontRightDrive", frontRight));
		hardware.add(new SelfCheckingSparkBase("BackRightDrive", backRight));
		return hardware;
	}
}
