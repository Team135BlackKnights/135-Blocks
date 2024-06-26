// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class TankIOTalonFXNavx implements TankIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private final TalonFX leftLeader = new TalonFX(
			DriveConstants.kFrontLeftDrivePort);
	private final TalonFX leftFollower = new TalonFX(
			DriveConstants.kBackLeftDrivePort);
	private final TalonFX rightLeader = new TalonFX(
			DriveConstants.kFrontRightDrivePort);
	private final TalonFX rightFollower = new TalonFX(
			DriveConstants.kBackRightDrivePort);
	private final StatusSignal<Double> leftPosition = leftLeader.getPosition();
	private final StatusSignal<Double> leftVelocity = leftLeader.getVelocity();
	private final StatusSignal<Double> leftAppliedVolts = leftLeader
			.getMotorVoltage();
	private final StatusSignal<Double> leftLeaderCurrent = leftLeader
			.getSupplyCurrent();
	private final StatusSignal<Double> leftFollowerCurrent = leftFollower
			.getSupplyCurrent();
	private final StatusSignal<Double> leftLeaderTemp = leftLeader
			.getDeviceTemp();
	private final StatusSignal<Double> leftFollowerTemp = leftFollower
			.getDeviceTemp();
	private final StatusSignal<Double> rightPosition = rightLeader.getPosition();
	private final StatusSignal<Double> rightVelocity = rightLeader.getVelocity();
	private final StatusSignal<Double> rightAppliedVolts = rightLeader
			.getMotorVoltage();
	private final StatusSignal<Double> rightLeaderCurrent = rightLeader
			.getSupplyCurrent();
	private final StatusSignal<Double> rightFollowerCurrent = rightFollower
			.getSupplyCurrent();
	private final StatusSignal<Double> rightLeaderTemp = rightLeader
			.getDeviceTemp();
	private final StatusSignal<Double> rightFollowerTemp = rightFollower
			.getDeviceTemp();
	private final AHRS navX = new AHRS(Port.kUSB);
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;

	public TankIOTalonFXNavx() {
		var config = new TalonFXConfiguration();
		config.CurrentLimits.SupplyCurrentLimit = DriveConstants.kMaxDriveCurrent;
		config.CurrentLimits.SupplyCurrentLimitEnable = true;
		config.MotorOutput.Inverted = DriveConstants.kFrontLeftDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
		config.Slot0.kP = KP;
		config.Slot0.kD = KD;
		leftLeader.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kBackLeftDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		leftFollower.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kFrontRightDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		rightLeader.getConfigurator().apply(config);
		config.MotorOutput.Inverted = DriveConstants.kBackRightDriveReversed
				? InvertedValue.CounterClockwise_Positive
				: InvertedValue.Clockwise_Positive;
		rightFollower.getConfigurator().apply(config);
		leftFollower.setControl(new Follower(leftLeader.getDeviceID(),
				DriveConstants.kBackLeftDriveReversed));
		rightFollower.setControl(new Follower(rightLeader.getDeviceID(),
				DriveConstants.kBackRightDriveReversed));
		BaseStatusSignal.setUpdateFrequencyForAll(100.0, leftPosition,
				rightPosition); // Required for odometry, use faster rate
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, leftVelocity,
				leftAppliedVolts, leftLeaderCurrent, leftFollowerCurrent,
				leftLeaderTemp, leftFollowerTemp, rightVelocity, rightAppliedVolts,
				rightLeaderCurrent, rightFollowerCurrent, rightLeaderTemp,
				rightFollowerTemp);
		leftLeader.optimizeBusUtilization();
		leftFollower.optimizeBusUtilization();
		rightLeader.optimizeBusUtilization();
		rightFollower.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(TankIOInputs inputs) {
		BaseStatusSignal.refreshAll(leftPosition, leftVelocity, leftAppliedVolts,
				leftLeaderCurrent, leftFollowerCurrent, leftLeaderTemp,
				leftFollowerTemp, rightPosition, rightVelocity, rightAppliedVolts,
				rightLeaderCurrent, rightFollowerCurrent, rightLeaderTemp,
				rightFollowerTemp);
		inputs.leftPositionRad = Units
				.rotationsToRadians(leftPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftVelocityRadPerSec = Units
				.rotationsToRadians(leftVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
		inputs.leftCurrentAmps = new double[] {
				leftLeaderCurrent.getValueAsDouble(),
				leftFollowerCurrent.getValueAsDouble()
		};
		inputs.frontLeftDriveTemp = leftLeaderTemp.getValueAsDouble();
		inputs.backLeftDriveTemp = leftFollowerTemp.getValueAsDouble();
		inputs.rightPositionRad = Units
				.rotationsToRadians(rightPosition.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightVelocityRadPerSec = Units
				.rotationsToRadians(rightVelocity.getValueAsDouble()) / GEAR_RATIO;
		inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
		inputs.rightCurrentAmps = new double[] {
				rightLeaderCurrent.getValueAsDouble(),
				rightFollowerCurrent.getValueAsDouble()
		};
		inputs.frontRightDriveTemp = rightLeaderTemp.getValueAsDouble();
		inputs.backRightDriveTemp = rightFollowerTemp.getValueAsDouble();
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
	public void setVoltage(double leftVolts, double rightVolts) {
		leftLeader.setControl(new VoltageOut(leftVolts));
		rightLeader.setControl(new VoltageOut(rightVolts));
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		leftLeader.setControl(new VelocityVoltage(
				Units.radiansToRotations(leftRadPerSec * GEAR_RATIO), 0.0, true,
				leftFFVolts, 0, false, false, false));
		rightLeader.setControl(new VelocityVoltage(
				Units.radiansToRotations(rightRadPerSec * GEAR_RATIO), 0.0, true,
				rightFFVolts, 0, false, false, false));
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingNavX2("IMU", navX));
		hardware.add(new SelfCheckingTalonFX("FrontLeftDrive", leftLeader));
		hardware.add(new SelfCheckingTalonFX("BackLeftDrive", leftFollower));
		hardware.add(new SelfCheckingTalonFX("FrontRightDrive", rightLeader));
		hardware.add(new SelfCheckingTalonFX("BackRightDrive", rightFollower));
		return hardware;
	}
}
