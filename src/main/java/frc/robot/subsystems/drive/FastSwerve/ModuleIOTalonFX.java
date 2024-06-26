// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.FastSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingTalonFX;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;

/**
 * Module IO implementation for Talon FX drive motor controller, Talon FX turn
 * motor controller, and CANcoder
 * <p>
 * NOTE: This implementation should be used as a starting point and adapted to
 * different hardware configurations (e.g. If using an analog encoder, copy from
 * "ModuleIOSparkMax")
 * <p>
 * To calibrate the absolute encoder offsets, point the modules straight (such
 * that forward motion on the drive motor will propel the robot forward) and
 * copy the reported values from the absolute encoders using AdvantageScope.
 * These values are logged under "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOTalonFX implements ModuleIO {
	private final TalonFX driveTalon;
	private final TalonFX turnTalon;
	private final CANcoder cancoder;
	private final String driveName;
	private final String turnName;
	private final Queue<Double> timestampQueue;
	private final StatusSignal<Double> drivePosition;
	private final Queue<Double> drivePositionQueue;
	private final StatusSignal<Double> driveVelocity;
	private final StatusSignal<Double> driveAppliedVolts;
	private final StatusSignal<Double> driveCurrent;
	private final StatusSignal<Double> driveTemp;
	private final StatusSignal<Double> turnAbsolutePosition;
	private final StatusSignal<Double> turnPosition;
	private final Queue<Double> turnPositionQueue;
	private final StatusSignal<Double> turnVelocity;
	private final StatusSignal<Double> turnAppliedVolts;
	private final StatusSignal<Double> turnCurrent;
	private final StatusSignal<Double> turnTemp;
	// Gear ratios for SDS MK4i L2, adjust as necessary
	private static final double DRIVE_GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double TURN_GEAR_RATIO = DriveConstants.TrainConstants.kTurningMotorGearRatio;
	private final boolean isTurnMotorInverted;
	private final boolean isDriveMotorInverted;
	private final Rotation2d absoluteEncoderOffset;

	public ModuleIOTalonFX(int index) {
		switch (index) {
		case 0:
			driveTalon = new TalonFX(DriveConstants.kFrontLeftDrivePort);
			turnTalon = new TalonFX(DriveConstants.kFrontLeftTurningPort);
			cancoder = new CANcoder(DriveConstants.kFrontLeftAbsEncoderPort);
			driveName = "FrontLeftDrive";
			turnName = "FrontLeftTurn";
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kFrontLeftAbsEncoderOffsetRad);
			isDriveMotorInverted = DriveConstants.kFrontLeftDriveReversed;
			isTurnMotorInverted = DriveConstants.kFrontLeftTurningReversed;
			break;
		case 1:
			driveTalon = new TalonFX(DriveConstants.kFrontRightDrivePort);
			turnTalon = new TalonFX(DriveConstants.kFrontRightTurningPort);
			cancoder = new CANcoder(DriveConstants.kFrontRightAbsEncoderPort);
			driveName = "FrontRightDrive";
			turnName = "FrontRightTurn";
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kFrontRightAbsEncoderOffsetRad);
			isDriveMotorInverted = DriveConstants.kFrontRightDriveReversed;
			isTurnMotorInverted = DriveConstants.kFrontRightTurningReversed;
			break;
		case 2:
			driveTalon = new TalonFX(DriveConstants.kBackLeftDrivePort);
			turnTalon = new TalonFX(DriveConstants.kBackLeftTurningPort);
			cancoder = new CANcoder(DriveConstants.kBackLeftAbsEncoderPort);
			driveName = "BackLeftDrive";
			turnName = "BackLeftTurn";
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kBackLeftAbsEncoderOffsetRad);
			isDriveMotorInverted = DriveConstants.kBackLeftDriveReversed;
			isTurnMotorInverted = DriveConstants.kBackLeftTurningReversed;
			break;
		case 3:
			driveTalon = new TalonFX(DriveConstants.kBackRightDrivePort);
			turnTalon = new TalonFX(DriveConstants.kBackRightTurningPort);
			driveName = "BackRightDrive";
			turnName = "BackRightTurn";
			cancoder = new CANcoder(DriveConstants.kBackRightAbsEncoderPort);
			absoluteEncoderOffset = new Rotation2d(
					DriveConstants.kBackRightAbsEncoderOffsetRad);
			isDriveMotorInverted = DriveConstants.kBackRightDriveReversed;
			isTurnMotorInverted = DriveConstants.kBackRightTurningReversed;
			break;
		default:
			throw new RuntimeException("Invalid module index");
		}
		var driveConfig = new TalonFXConfiguration();
		driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.kMaxDriveCurrent;
		driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		driveTalon.getConfigurator().apply(driveConfig);
		setDriveBrakeMode(true);
		var turnConfig = new TalonFXConfiguration();
		turnConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.kMaxTurnCurrent;
		turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
		turnTalon.getConfigurator().apply(turnConfig);
		setTurnBrakeMode(true);
		cancoder.getConfigurator().apply(new CANcoderConfiguration());
		timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
		drivePosition = driveTalon.getPosition();
		drivePositionQueue = PhoenixOdometryThread.getInstance()
				.registerSignal(driveTalon, driveTalon.getPosition());
		driveVelocity = driveTalon.getVelocity();
		driveAppliedVolts = driveTalon.getMotorVoltage();
		driveCurrent = driveTalon.getSupplyCurrent();
		driveTemp = driveTalon.getDeviceTemp();
		turnAbsolutePosition = cancoder.getAbsolutePosition();
		turnPosition = turnTalon.getPosition();
		turnPositionQueue = PhoenixOdometryThread.getInstance()
				.registerSignal(turnTalon, turnTalon.getPosition());
		turnVelocity = turnTalon.getVelocity();
		turnAppliedVolts = turnTalon.getMotorVoltage();
		turnCurrent = turnTalon.getSupplyCurrent();
		turnTemp = turnTalon.getDeviceTemp();
		BaseStatusSignal.setUpdateFrequencyForAll(Module.ODOMETRY_FREQUENCY,
				drivePosition, turnPosition);
		BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity,
				driveAppliedVolts, driveCurrent, driveTemp, turnAbsolutePosition,
				turnVelocity, turnAppliedVolts, turnCurrent, turnTemp);
		driveTalon.optimizeBusUtilization();
		turnTalon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		BaseStatusSignal.refreshAll(drivePosition, driveVelocity,
				driveAppliedVolts, driveCurrent, driveTemp, turnAbsolutePosition,
				turnPosition, turnVelocity, turnAppliedVolts, turnCurrent,
				turnTemp);
		inputs.drivePositionRad = Units.rotationsToRadians(
				drivePosition.getValueAsDouble()) / DRIVE_GEAR_RATIO;
		inputs.driveVelocityRadPerSec = Units.rotationsToRadians(
				driveVelocity.getValueAsDouble()) / DRIVE_GEAR_RATIO;
		inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
		inputs.driveCurrentAmps = new double[] { driveCurrent.getValueAsDouble()
		};
		inputs.driveMotorTemp = driveTemp.getValueAsDouble();
		inputs.turnAbsolutePosition = Rotation2d
				.fromRotations(turnAbsolutePosition.getValueAsDouble())
				.minus(absoluteEncoderOffset);
		inputs.turnPosition = Rotation2d
				.fromRotations(turnPosition.getValueAsDouble() / TURN_GEAR_RATIO);
		inputs.turnVelocityRadPerSec = Units.rotationsToRadians(
				turnVelocity.getValueAsDouble()) / TURN_GEAR_RATIO;
		inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
		inputs.turnCurrentAmps = new double[] { turnCurrent.getValueAsDouble()
		};
		inputs.turnMotorTemp = turnTemp.getValueAsDouble();
		inputs.odometryTimestamps = timestampQueue.stream()
				.mapToDouble((Double value) -> value).toArray();
		inputs.odometryDrivePositionsRad = drivePositionQueue.stream()
				.mapToDouble((Double value) -> Units.rotationsToRadians(value)
						/ DRIVE_GEAR_RATIO)
				.toArray();
		inputs.odometryTurnPositions = turnPositionQueue.stream().map(
				(Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
				.toArray(Rotation2d[]::new);
		timestampQueue.clear();
		drivePositionQueue.clear();
		turnPositionQueue.clear();
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnTalon.setControl(new VoltageOut(volts));
	}

	@Override
	public void setDriveBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		driveTalon.getConfigurator().apply(config);
	}

	@Override
	public void setTurnBrakeMode(boolean enable) {
		var config = new MotorOutputConfigs();
		config.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive
				: InvertedValue.CounterClockwise_Positive;
		config.NeutralMode = enable ? NeutralModeValue.Brake
				: NeutralModeValue.Coast;
		turnTalon.getConfigurator().apply(config);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingTalonFX(driveName, driveTalon));
		hardware.add(new SelfCheckingTalonFX(turnName, turnTalon));
		return hardware;
	}
}
