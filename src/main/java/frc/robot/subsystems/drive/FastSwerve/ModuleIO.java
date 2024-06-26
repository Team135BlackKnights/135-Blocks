// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.math.geometry.Rotation2d;

import org.littletonrobotics.junction.AutoLog;
import frc.robot.utils.selfCheck.SelfChecking;
import java.util.List;
import java.util.ArrayList;

public interface ModuleIO {
	@AutoLog
	public static class ModuleIOInputs {
		public double drivePositionRad = 0.0;
		public double driveVelocityRadPerSec = 0.0;
		public double driveAppliedVolts = 0.0;
		public double[] driveCurrentAmps = new double[] {};
		public double driveMotorTemp = 0.0;
		public Rotation2d turnAbsolutePosition = new Rotation2d();
		public Rotation2d turnPosition = new Rotation2d();
		public double turnVelocityRadPerSec = 0.0;
		public double turnAppliedVolts = 0.0;
		public double[] turnCurrentAmps = new double[] {};
		public double turnMotorTemp = 0.0;
		public double[] odometryTimestamps = new double[] {};
		public double[] odometryDrivePositionsRad = new double[] {};
		public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
	}

	/** Updates the set of loggable inputs. */
	public default void updateInputs(ModuleIOInputs inputs) {}

	/** Run the drive motor at the specified voltage. */
	public default void setDriveVoltage(double volts) {}

	/** Run the turn motor at the specified voltage. */
	public default void setTurnVoltage(double volts) {}

	/** Enable or disable brake mode on the drive motor. */
	public default void setDriveBrakeMode(boolean enable) {}

	/** Enable or disable brake mode on the turn motor. */
	public default void setTurnBrakeMode(boolean enable) {}

	/**
	 * Get a list of the SelfChecking interface for all hardware in that
	 * implementation
	 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
