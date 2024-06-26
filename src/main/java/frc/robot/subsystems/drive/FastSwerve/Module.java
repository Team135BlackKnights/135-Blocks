package frc.robot.subsystems.drive.FastSwerve;

import java.util.List;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.LoggableTunedNumber;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;

import org.littletonrobotics.junction.Logger;

public class Module {
	private static final LoggableTunedNumber drivekP = new LoggableTunedNumber(
			"Drive/Module/DrivekP",
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer
					.getP());
	private static final LoggableTunedNumber drivekD = new LoggableTunedNumber(
			"Drive/Module/DrivekD",
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer
					.getD());
	private static final LoggableTunedNumber drivekS = new LoggableTunedNumber(
			"Drive/Module/DrivekS",
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer
					.getKs());
	private static final LoggableTunedNumber drivekV = new LoggableTunedNumber(
			"Drive/Module/DrivekV",
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer
					.getKv());
	private static final LoggableTunedNumber turnkP = new LoggableTunedNumber(
			"Drive/Module/TurnkP",
			DriveConstants.TrainConstants.overallTurningMotorConstantContainer
					.getP());
	private static final LoggableTunedNumber turnkD = new LoggableTunedNumber(
			"Drive/Module/TurnkD",
			DriveConstants.TrainConstants.overallTurningMotorConstantContainer
					.getD());
	private SwerveModuleState setpointState = new SwerveModuleState();
	private final int index;
	private final ModuleIO io;
	private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
	private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer
					.getKs(),
			DriveConstants.TrainConstants.overallDriveMotorConstantContainer
					.getKv(),
			0.0);

	public Module(ModuleIO io, int index) {
		this.io = io;
		this.index = index;
	}

	/** Called while blocking odometry thread */
	public void updateInputs() {
		io.updateInputs(inputs);
		Logger.processInputs("Drive/Module" + index, inputs);
		// Update ff and controllers
		LoggableTunedNumber.ifChanged(hashCode(),
				() -> ff = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(),
						0),
				drivekS, drivekV);
		LoggableTunedNumber.ifChanged(hashCode(),
				() -> io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP,
				drivekD);
		LoggableTunedNumber.ifChanged(hashCode(),
				() -> io.setTurnPID(turnkP.get(), 0, turnkD.get()), turnkP, turnkD);
	}

	/** Runs to {@link SwerveModuleState} */
	public void runSetpoint(SwerveModuleState setpoint,
			SwerveModuleState torqueFF) {
		setpointState = setpoint;
		double wheelTorqueNm = torqueFF.speedMetersPerSecond; // Using SwerveModuleState for torque for easy logging
		io.runDriveVelocitySetpoint(
				setpoint.speedMetersPerSecond
						/ (DriveConstants.TrainConstants.kWheelDiameter / 2),
				ff.calculate(setpoint.speedMetersPerSecond
						/ (DriveConstants.TrainConstants.kWheelDiameter / 2))
						+ ((wheelTorqueNm
								/ DriveConstants.TrainConstants.kDriveMotorGearRatio)
								* DriveConstants.TrainConstants.kT));
		io.runTurnPositionSetpoint(setpoint.angle.getRadians());
	}

	/**
	 * Runs characterization volts or amps depending on using voltage or current
	 * control.
	 */
	public void runCharacterization(double turnSetpointRads, double input) {
		io.runTurnPositionSetpoint(turnSetpointRads);
		io.runCharacterization(input);
	}

	/** Sets brake mode to {@code enabled}. */
	public void setBrakeMode(boolean enabled) {
		io.setDriveBrakeMode(enabled);
		io.setTurnBrakeMode(enabled);
	}

	/** Stops motors. */
	public void stop() { io.stop(); }

	/** Get all latest {@link SwerveModulePosition}'s from last cycle. */
	public SwerveModulePosition[] getModulePositions() {
		int minOdometryPositions = Math.min(
				inputs.odometryDrivePositionsMeters.length,
				inputs.odometryTurnPositions.length);
		SwerveModulePosition[] positions = new SwerveModulePosition[minOdometryPositions];
		for (int i = 0; i < minOdometryPositions; i++) {
			positions[i] = new SwerveModulePosition(
					inputs.odometryDrivePositionsMeters[i],
					inputs.odometryTurnPositions[i]);
		}
		return positions;
	}

	/** Get turn angle of module as {@link Rotation2d}. */
	public Rotation2d getAngle() {
		return inputs.turnAbsolutePosition;
	}

	/** Get position of wheel rotations in radians */
	public double getPositionRads() {
		return inputs.drivePositionRads;
	}

	/** Get position of wheel in meters. */
	public double getPositionMeters() {
		return inputs.drivePositionRads
				* (DriveConstants.TrainConstants.kWheelDiameter / 2);
	}

	/** Get velocity of wheel in m/s. */
	public double getVelocityMetersPerSec() {
		return inputs.driveVelocityRadsPerSec
				* (DriveConstants.TrainConstants.kWheelDiameter / 2);
	}

	/** Get current {@link SwerveModulePosition} of module. */
	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getPositionMeters(), getAngle());
	}

	/** Get current {@link SwerveModuleState} of module. */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
	}

	/** Get velocity of drive wheel for characterization */
	public double getCharacterizationVelocity() {
		return inputs.driveVelocityRadsPerSec;
	}

	/** Returns the temp of the DRIVE motor in C */
	public double getDriveMotorTemp() {
		return inputs.driveMotorTemp;
	}

	public double getCurrent() {
		return inputs.driveSupplyCurrentAmps + inputs.turnSupplyCurrentAmps;
	}

	/** Returns the temp of the DRIVE motor in C */
	public double getTurnMotorTemp() {
		return inputs.turnMotorTemp;
	}

	public SwerveModuleState getSetpointState() { return setpointState; }

	public List<SelfChecking> getSelfCheckingHardware() {
		return io.getSelfCheckingHardware();
	}
}
