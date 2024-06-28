// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.FastSwerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;

/**
 * Physics sim implementation of module IO.
 * <p>
 * Uses two dc motor sims for the drive and turn motors, with the absolute
 * position initialized to a random value. The dc motor sims are not physically
 * accurate, but provide a decent approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
	private static final double LOOP_PERIOD_SECS = 0.02;
	private DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1),
			DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.005);
	private DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1),
			DriveConstants.TrainConstants.kTurningMotorGearRatio, 0.004);
	private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(
			Math.random() * 2.0 * Math.PI);
	private double driveAppliedVolts = 0.0;
	private double turnAppliedVolts = 0.0;

	@Override
	public void updateInputs(ModuleIOInputs inputs) {
		driveSim.update(LOOP_PERIOD_SECS);
		turnSim.update(LOOP_PERIOD_SECS);
		inputs.drivePositionRad = driveSim.getAngularPositionRad();
		inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
		inputs.driveAppliedVolts = driveAppliedVolts;
		inputs.driveCurrentAmps = new double[] {
				Math.abs(driveSim.getCurrentDrawAmps())
		};
		inputs.turnAbsolutePosition = new Rotation2d(
				turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
		inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
		inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
		inputs.turnAppliedVolts = turnAppliedVolts;
		inputs.turnCurrentAmps = new double[] {
				Math.abs(turnSim.getCurrentDrawAmps())
		};
		inputs.odometryTimestamps = new double[] { Logger.getTimestamp()
		};
		inputs.odometryDrivePositionsRad = new double[] { inputs.drivePositionRad
		};
		inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition
		};
	}

	@Override
	public void setDriveVoltage(double volts) {
		driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		driveSim.setInputVoltage(driveAppliedVolts);
	}

	@Override
	public void setTurnVoltage(double volts) {
		turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
		turnSim.setInputVoltage(turnAppliedVolts);
	}
}
