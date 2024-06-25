// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
package frc.robot.subsystems.drive.Tank;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingPigeon2;
import frc.robot.utils.selfCheck.SelfCheckingSparkBase;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of
 * "CANSparkMax" with "CANSparkFlex".
 */
public class DriveIOSparkMax implements DriveIO {
	private static final double GEAR_RATIO = DriveConstants.TrainConstants.kDriveMotorGearRatio;
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private final CANSparkMax leftLeader = new CANSparkMax(
			DriveConstants.kFrontLeftDrivePort, MotorType.kBrushless);
	private final CANSparkMax rightLeader = new CANSparkMax(
			DriveConstants.kFrontRightDrivePort, MotorType.kBrushless);
	private final CANSparkMax leftFollower = new CANSparkMax(
			DriveConstants.kBackLeftDrivePort, MotorType.kBrushless);
	private final CANSparkMax rightFollower = new CANSparkMax(
			DriveConstants.kBackRightDrivePort, MotorType.kBrushless);
	private final RelativeEncoder leftEncoder = leftLeader.getEncoder();
	private final RelativeEncoder rightEncoder = rightLeader.getEncoder();
	private final SparkPIDController leftPID = leftLeader.getPIDController();
	private final SparkPIDController rightPID = rightLeader.getPIDController();
	private final Pigeon2 pigeon = new Pigeon2(30);
	private final StatusSignal<Double> yaw = pigeon.getYaw();
	private final StatusSignal<Double> accelX = pigeon
			.getAccelerationX();
	private final StatusSignal<Double> accelY = pigeon
			.getAccelerationY();
	private double last_world_linear_accel_x;
	private double last_world_linear_accel_y;
	public DriveIOSparkMax() {
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
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(100.0);
		pigeon.optimizeBusUtilization();
	}

	@Override
	public void updateInputs(DriveIOInputs inputs) {
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
				inputs.gyroConnected = BaseStatusSignal.refreshAll(yaw,accelX,accelY)
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
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			inputs.collisionDetected = true;
		}
		inputs.collisionDetected = false;
	}

	@Override
	public void setVoltage(double leftVolts, double rightVolts) {
		leftLeader.setVoltage(leftVolts);
		rightLeader.setVoltage(rightVolts);
	}
	@Override
	public void reset(){
		pigeon.reset();
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
		hardware.add(new SelfCheckingPigeon2("IMU", pigeon));
		hardware.add(new SelfCheckingSparkBase("FrontLeftDrive", leftLeader));
		hardware.add(new SelfCheckingSparkBase("BackLeftDrive", leftFollower));
		hardware.add(new SelfCheckingSparkBase("FrontRightDrive", rightLeader));
		hardware.add(new SelfCheckingSparkBase("BackRightDrive", rightFollower));
		return hardware;
	}
}
