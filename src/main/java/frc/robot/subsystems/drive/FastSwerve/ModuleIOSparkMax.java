// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!

package frc.robot.subsystems.drive.FastSwerve;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.SparkAnalogSensor.Mode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingSparkBase;

import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.ArrayList;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
  private static final double TURN_GEAR_RATIO = DriveConstants.TrainConstants.kTurningMotorGearRatio;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;
  private final String driveName;
  private final String turnName;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final SparkAnalogSensor turnAbsoluteEncoder;
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted;
  private final boolean isDriveMotorInverted;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningPort, MotorType.kBrushless);
		  driveName = "FrontLeftDrive";
		  turnName = "FrontLeftTurn";
        turnAbsoluteEncoder = turnSparkMax.getAnalog(Mode.kAbsolute);
        absoluteEncoderOffset = new Rotation2d(DriveConstants.kFrontLeftAbsEncoderOffsetRad);
		  isDriveMotorInverted = DriveConstants.kFrontLeftDriveReversed;
		  isTurnMotorInverted = DriveConstants.kFrontLeftTurningReversed;
        break;
      case 1:
        driveSparkMax = new CANSparkMax(DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kFrontRightTurningPort, MotorType.kBrushless);
		  driveName = "FrontRightDrive";
		  turnName = "FrontRightTurn";
        turnAbsoluteEncoder = turnSparkMax.getAnalog(Mode.kAbsolute);
        absoluteEncoderOffset = new Rotation2d(DriveConstants.kFrontRightAbsEncoderOffsetRad);
		  isDriveMotorInverted = DriveConstants.kFrontRightDriveReversed;
		  isTurnMotorInverted = DriveConstants.kFrontRightTurningReversed;
        break;
      case 2:
        driveSparkMax = new CANSparkMax(DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kBackLeftTurningPort, MotorType.kBrushless);
		  driveName = "BackLeftDrive";
		  turnName = "BackLeftTurn";
        turnAbsoluteEncoder = turnSparkMax.getAnalog(Mode.kAbsolute);
        absoluteEncoderOffset = new Rotation2d(DriveConstants.kBackLeftAbsEncoderOffsetRad);
		  isDriveMotorInverted = DriveConstants.kBackLeftDriveReversed;
		  isTurnMotorInverted = DriveConstants.kBackLeftTurningReversed;
        break;
      case 3:
        driveSparkMax = new CANSparkMax(DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(DriveConstants.kBackRightTurningPort, MotorType.kBrushless);
		  driveName = "BackRightDrive";
		  turnName = "BackRightTurn";
        turnAbsoluteEncoder = turnSparkMax.getAnalog(Mode.kAbsolute);
        absoluteEncoderOffset = new Rotation2d(DriveConstants.kBackRightAbsEncoderOffsetRad);
		  isDriveMotorInverted = DriveConstants.kBackRightDriveReversed;
		  isTurnMotorInverted = DriveConstants.kBackRightTurningReversed;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
	 driveSparkMax.setInverted(isDriveMotorInverted);
    driveSparkMax.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
    turnSparkMax.setSmartCurrentLimit(DriveConstants.kMaxTurnCurrent);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = driveEncoder.getPosition();
                  if (driveSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () -> {
                  double value = turnRelativeEncoder.getPosition();
                  if (turnSparkMax.getLastError() == REVLibError.kOk) {
                    return OptionalDouble.of(value);
                  } else {
                    return OptionalDouble.empty();
                  }
                });

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};
	 inputs.driveMotorTemp = driveSparkMax.getMotorTemperature();
    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage3V3() * 2.0 * Math.PI) //May be RobotController.getVoltage5V
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
    inputs.turnMotorTemp = turnSparkMax.getMotorTemperature();
    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
  @Override
  public List<SelfChecking> getSelfCheckingHardware(){
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingSparkBase(driveName, driveSparkMax));
		hardware.add(new SelfCheckingSparkBase(turnName, driveSparkMax));
		return hardware;
  }
}
