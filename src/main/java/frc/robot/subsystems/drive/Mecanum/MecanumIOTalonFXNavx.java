// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingNavX2;
import frc.robot.utils.selfCheck.drive.SelfCheckingTalonFX;

public class MecanumIOTalonFXNavx implements MecanumIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private final TalonFX frontLeft = new TalonFX(
			DriveConstants.kFrontLeftDrivePort);
	private final TalonFX backLeft = new TalonFX(
			DriveConstants.kBackLeftDrivePort);
	private final TalonFX frontRight = new TalonFX(
			DriveConstants.kFrontRightDrivePort);
	private final TalonFX backRight = new TalonFX(
			DriveConstants.kBackRightDrivePort);
	private final StatusSignal<Double> frontLeftPosition = frontLeft
			.getPosition();
	private final StatusSignal<Double> frontLeftVelocity = frontLeft
			.getVelocity();
	private final StatusSignal<Double> frontLeftAppliedVolts = frontLeft
			.getMotorVoltage();
	private final StatusSignal<Double> frontLeftCurrent = frontLeft
			.getSupplyCurrent();
	private final StatusSignal<Double> frontLeftTemp = frontLeft.getDeviceTemp();
	private final StatusSignal<Double> frontRightPosition = frontRight
			.getPosition();
	private final StatusSignal<Double> frontRightVelocity = frontRight
			.getVelocity();
	private final StatusSignal<Double> frontRightAppliedVolts = frontRight
			.getMotorVoltage();
	private final StatusSignal<Double> frontRightCurrent = frontRight
			.getSupplyCurrent();
	private final StatusSignal<Double> frontRightTemp = frontRight
			.getDeviceTemp();
	private final StatusSignal<Double> backLeftPosition = backLeft.getPosition();
	private final StatusSignal<Double> backLeftVelocity = backLeft.getVelocity();
	private final StatusSignal<Double> backLeftAppliedVolts = backLeft
			.getMotorVoltage();
	private final StatusSignal<Double> backLeftCurrent = backLeft
			.getSupplyCurrent();
	private final StatusSignal<Double> backLeftTemp = backLeft.getDeviceTemp();
	private final StatusSignal<Double> backRightPosition = backRight
			.getPosition();
	private final StatusSignal<Double> backRightVelocity = backRight
			.getVelocity();
	private final StatusSignal<Double> backRightAppliedVolts = backRight
			.getMotorVoltage();
	private final StatusSignal<Double> backRightCurrent = backRight
			.getSupplyCurrent();
	private final StatusSignal<Double> backRightTemp = backRight.getDeviceTemp();
	private final AHRS navX = new AHRS(Port.kUSB);
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;

	public MecanumIOTalonFXNavx() {
		var config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimit = DriveConstants.kMaxDriveCurrent;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.MotorOutput.Inverted = DriveConstants.kFrontLeftDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.Slot0.kP = KP;
		config.Slot0.kD = KD;
		frontLeft.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kBackLeftDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		backLeft.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kFrontRightDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		frontRight.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kBackRightDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		backRight.getConfigurator().apply(config);
		BaseStatusSignal.setUpdateFrequencyForAll(100.0, frontLeftPosition,
				frontRightPosition, backLeftPosition, backRightPosition); // Required for odometry, use faster rate
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, frontLeftVelocity,
				frontLeftAppliedVolts, frontLeftCurrent, frontLeftTemp,
				frontRightVelocity, frontRightAppliedVolts, frontRightCurrent,
				frontRightTemp, backLeftVelocity, backLeftAppliedVolts,
				backLeftCurrent, backLeftTemp, backRightVelocity,
				backRightAppliedVolts, backRightCurrent, backRightTemp);
		frontLeft.optimizeBusUtilization();
		backLeft.optimizeBusUtilization();
		frontRight.optimizeBusUtilization();
		backRight.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(MecanumIOInputs inputs) {
		BaseStatusSignal.refreshAll(frontLeftVelocity, frontLeftAppliedVolts,
				frontLeftCurrent, frontLeftTemp, frontRightVelocity,
				frontRightAppliedVolts, frontRightCurrent, frontRightTemp,
				backLeftVelocity, backLeftAppliedVolts, backLeftCurrent,
				backLeftTemp, backRightVelocity, backRightAppliedVolts,
				backRightCurrent, backRightTemp, frontLeftPosition,
				frontRightPosition, backLeftPosition, backRightPosition);
		inputs.leftFrontPositionRad = Units.rotationsToRadians(
				frontLeftPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftFrontVelocityRadPerSec = Units.rotationsToRadians(
				frontLeftVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftFrontAppliedVolts = frontLeftAppliedVolts.getValueAsDouble();
		inputs.leftBackPositionRad = Units.rotationsToRadians(
				backLeftPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftBackVelocityRadPerSec = Units.rotationsToRadians(
				backLeftVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftBackAppliedVolts = backLeftAppliedVolts.getValueAsDouble();
		inputs.leftCurrentAmps = new double[] {
				frontLeftCurrent.getValueAsDouble(),
				backLeftCurrent.getValueAsDouble()
		};
		inputs.frontLeftDriveTemp = frontLeftTemp.getValueAsDouble();
		inputs.backLeftDriveTemp = backLeftTemp.getValueAsDouble();
		inputs.rightFrontPositionRad = Units.rotationsToRadians(
				frontRightPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightFrontVelocityRadPerSec = Units.rotationsToRadians(
				frontRightVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightFrontAppliedVolts = frontRightAppliedVolts.getValueAsDouble();
		inputs.rightBackPositionRad = Units.rotationsToRadians(
				backRightPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightBackVelocityRadPerSec = Units.rotationsToRadians(
				backRightVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightBackAppliedVolts = backRightAppliedVolts.getValueAsDouble();
		inputs.rightCurrentAmps = new double[] {
				frontRightCurrent.getValueAsDouble(),
				backRightCurrent.getValueAsDouble()
		};
		inputs.frontRightDriveTemp = frontRightTemp.getValueAsDouble();
		inputs.backRightDriveTemp = backRightTemp.getValueAsDouble();
		inputs.gyroConnected = navX.isConnected();
		inputs.gyroYaw = navX.getRotation2d();
		double curr_world_linear_accel_x = navX.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = navX.getWorldLinearAccelY();
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
	public void reset() { navX.reset(); }

	@Override
	public void setVoltage(double frontLeftVolts, double frontRightVolts,
			double backLeftVolts, double backRightVolts) {
		frontLeft.setControl(new VoltageOut(frontLeftVolts));
		frontRight.setControl(new VoltageOut(frontRightVolts));
		backLeft.setControl(new VoltageOut(backLeftVolts));
		backRight.setControl(new VoltageOut(backRightVolts));
	}

	@Override
	public void setVelocity(double frontLeftRadPerSec,
			double frontRightRadPerSec, double backLeftRadPerSec,
			double backRightRadPerSec, double frontLeftFFVolts,
			double frontRightFFVolts, double backLeftFFVolts,
			double backRightFFVolts) {
		frontLeft.setControl(new VelocityVoltage(
				Units.radiansToRotations(frontLeftRadPerSec * GEAR_RATIO), 0.0,
				true, frontLeftFFVolts, 0, false, false, false));
		frontRight.setControl(new VelocityVoltage(
				Units.radiansToRotations(frontRightRadPerSec * GEAR_RATIO), 0.0,
				true, frontRightFFVolts, 0, false, false, false));
		backLeft.setControl(new VelocityVoltage(
				Units.radiansToRotations(backLeftRadPerSec * GEAR_RATIO), 0.0, true,
				backLeftFFVolts, 0, false, false, false));
		backRight.setControl(new VelocityVoltage(
				Units.radiansToRotations(backRightRadPerSec * GEAR_RATIO), 0.0,
				true, backRightFFVolts, 0, false, false, false));
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingNavX2("IMU", navX));
		hardware.add(new SelfCheckingTalonFX("FrontLeftDrive", frontLeft));
		hardware.add(new SelfCheckingTalonFX("BackLeftDrive", backLeft));
		hardware.add(new SelfCheckingTalonFX("FrontRightDrive", frontRight));
		hardware.add(new SelfCheckingTalonFX("BackRightDrive", backRight));
		return hardware;
	}
}
