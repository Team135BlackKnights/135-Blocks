// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.MotorVendor;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.drive.SelfCheckingNavX2;
import frc.robot.utils.selfCheck.drive.SelfCheckingSparkBase;

public class TankIOSparkBaseNavx implements TankIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private final CANSparkBase leftLeader;
	private final CANSparkBase rightLeader;
	private final CANSparkBase leftFollower;
	private final CANSparkBase rightFollower;
	private final RelativeEncoder leftEncoder;
	private final RelativeEncoder rightEncoder;
	private final SparkPIDController leftPID;
	private final SparkPIDController rightPID;
	private final AHRS navX = new AHRS(Port.kUSB);
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;

	public TankIOSparkBaseNavx() {
		if (DriveConstants.robotMotorController == MotorVendor.NEO_SPARK_MAX) {
			leftLeader = new CANSparkMax(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			rightLeader = new CANSparkMax(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			leftFollower = new CANSparkMax(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			rightFollower = new CANSparkMax(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
		} else {
			leftLeader = new CANSparkFlex(DriveConstants.kFrontLeftDrivePort,
					MotorType.kBrushless);
			rightLeader = new CANSparkFlex(DriveConstants.kFrontRightDrivePort,
					MotorType.kBrushless);
			leftFollower = new CANSparkFlex(DriveConstants.kBackLeftDrivePort,
					MotorType.kBrushless);
			rightFollower = new CANSparkFlex(DriveConstants.kBackRightDrivePort,
					MotorType.kBrushless);
		}
		leftEncoder = leftLeader.getEncoder();
		rightEncoder = rightLeader.getEncoder();
		leftPID = leftLeader.getPIDController();
		rightPID = rightLeader.getPIDController();
		leftLeader.restoreFactoryDefaults();
		rightLeader.restoreFactoryDefaults();
		leftFollower.restoreFactoryDefaults();
		rightFollower.restoreFactoryDefaults();
		leftLeader.setCANTimeout(250);
		rightLeader.setCANTimeout(250);
		leftFollower.setCANTimeout(250);
		rightFollower.setCANTimeout(250);
		leftLeader.setInverted(DriveConstants.kFrontLeftDriveReversed);
		rightLeader.setInverted(DriveConstants.kFrontRightDriveReversed);
		leftFollower.follow(leftLeader, DriveConstants.kBackLeftDriveReversed);
		rightFollower.follow(rightLeader, DriveConstants.kBackRightDriveReversed);
		leftLeader.enableVoltageCompensation(12.0);
		rightLeader.enableVoltageCompensation(12.0);
		leftLeader.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		rightLeader.setSmartCurrentLimit(DriveConstants.kMaxDriveCurrent);
		leftPID.setP(KP);
		leftPID.setD(KD);
		rightPID.setP(KP);
		rightPID.setD(KD);
		leftLeader.burnFlash();
		rightLeader.burnFlash();
		leftFollower.burnFlash();
		rightFollower.burnFlash();
	}

	@Override
	public void updateInputs(TankIOInputs inputs) {
		inputs.leftPositionRad = Units
				.rotationsToRadians(leftEncoder.getPosition() / GEAR_RATIO);
		inputs.leftVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(
				leftEncoder.getVelocity() / GEAR_RATIO);
		inputs.leftAppliedVolts = leftLeader.getAppliedOutput()
				* leftLeader.getBusVoltage();
		inputs.leftCurrentAmps = new double[] { leftLeader.getOutputCurrent(),
				leftFollower.getOutputCurrent()
		};
		inputs.frontLeftDriveTemp = leftLeader.getMotorTemperature();
		inputs.backLeftDriveTemp = leftFollower.getMotorTemperature();
		inputs.rightPositionRad = Units
				.rotationsToRadians(rightEncoder.getPosition() / GEAR_RATIO);
		inputs.rightVelocityRadPerSec = Units
				.rotationsPerMinuteToRadiansPerSecond(
						rightEncoder.getVelocity() / GEAR_RATIO);
		inputs.rightAppliedVolts = rightLeader.getAppliedOutput()
				* rightLeader.getBusVoltage();
		inputs.rightCurrentAmps = new double[] { rightLeader.getOutputCurrent(),
				rightFollower.getOutputCurrent()
		};
		inputs.frontRightDriveTemp = rightLeader.getMotorTemperature();
		inputs.backRightDriveTemp = rightFollower.getMotorTemperature();
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
		leftLeader.setVoltage(leftVolts);
		rightLeader.setVoltage(rightVolts);
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		leftPID.setReference(
				Units.radiansPerSecondToRotationsPerMinute(
						leftRadPerSec * GEAR_RATIO),
				ControlType.kVelocity, 0, leftFFVolts);
		rightPID.setReference(
				Units.radiansPerSecondToRotationsPerMinute(
						rightRadPerSec * GEAR_RATIO),
				ControlType.kVelocity, 0, rightFFVolts);
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingNavX2("IMU", navX));
		hardware.add(new SelfCheckingSparkBase("FrontLeftDrive", leftLeader));
		hardware.add(new SelfCheckingSparkBase("BackLeftDrive", leftFollower));
		hardware.add(new SelfCheckingSparkBase("FrontRightDrive", rightLeader));
		hardware.add(new SelfCheckingSparkBase("BackRightDrive", rightFollower));
		return hardware;
	}
}
