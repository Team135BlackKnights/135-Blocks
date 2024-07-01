package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;

public class ModuleIOSim implements ModuleIO {
	private final DCMotorSim driveSim = new DCMotorSim(
			DCMotor.getKrakenX60Foc(1),
			DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.005);
	private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getKrakenX60Foc(1),
			DriveConstants.TrainConstants.kTurningMotorGearRatio, 0.004);
	private final PIDController driveFeedback = new PIDController(0.0, 0.0, 0.0,
			.02);
	private final PIDController turnFeedback = new PIDController(0.0, 0.0, 0.0,
			.02);
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;
	private final Rotation2d turnAbsoluteInitPosition;
	private boolean driveCoast = false;
	private SlewRateLimiter driveVoltsLimiter = new SlewRateLimiter(2.5);

	public ModuleIOSim(int index) {
		switch (index) {
		case 0:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		case 1:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		case 2:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		default:
			turnAbsoluteInitPosition = new Rotation2d(0);
			break;
		}
		turnFeedback.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		if (DriverStation.isDisabled()) {
			stop();
		}
		if (driveCoast && DriverStation.isDisabled()) {
			runDriveVolts(driveVoltsLimiter.calculate(driveAppliedVolts));
		} else {
			driveVoltsLimiter.reset(driveAppliedVolts);
		}
		driveSim.update(.02);
		turnSim.update(.02);
		inputs.drivePositionRads = driveSim.getAngularPositionRad();
		inputs.driveVelocityRadsPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveSupplyCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
		inputs.turnAbsolutePosition = new Rotation2d(
				turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
		inputs.turnPosition = Rotation2d
				.fromRadians(turnSim.getAngularPositionRad());
		inputs.turnVelocityRadsPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnSupplyCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
		inputs.odometryDrivePositionsMeters = new double[] {
				driveSim.getAngularPositionRad()
						* (DriveConstants.TrainConstants.kWheelDiameter / 2)
		};
		inputs.odometryTurnPositions = new Rotation2d[] {
				Rotation2d.fromRadians(turnSim.getAngularPositionRad())
		};
	}

	public void runDriveVolts(double volts) {
		driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		driveSim.setInputVoltage(driveAppliedVolts);
	}

	public void runTurnVolts(double volts) {
		turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		turnSim.setInputVoltage(turnAppliedVolts);
	}

	@Override
	public void runCharacterization(double input) { runDriveVolts(input); }

	@Override
	public void runDriveVelocitySetpoint(double velocityRadsPerSec,
			double feedForward) {
		runDriveVolts(
				driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(),
						velocityRadsPerSec) + feedForward);
	}

	@Override
	public void runTurnPositionSetpoint(double angleRads) {
		runTurnVolts(
				turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads));
	}

	@Override
	public void setDrivePID(double kP, double kI, double kD) {
		driveFeedback.setPID(kP, kI, kD);
	}

	@Override
	public void setTurnPID(double kP, double kI, double kD) {
		turnFeedback.setPID(kP, kI, kD);
	}

	@Override
	public void setDriveBrakeMode(boolean enable) { driveCoast = !enable; }

	@Override
	public void stop() {
		runDriveVolts(0.0);
		runTurnVolts(0.0);
	}
}
