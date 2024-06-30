package frc.robot.subsystems.state_space.SingleJointedArm;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.drive.SelfChecking;

public interface SingleJointedArmIO {
	@AutoLog
	public static class SingleJointedArmIOInputs{
		public double positionRad = 0.0;
		public double velocityRadPerSec = 0.0;
		public double armTemp = 0.0;
		public double appliedVolts = 0.0;
		public double[] currentAmps = new double[]{};
	}
	 /** Updates the set of loggable inputs. */
  public default void updateInputs(SingleJointedArmIOInputs inputs) {}
  /** Dumb control the Arm using volts, no State Space*/
  public default void setVoltage(double volts){}
  /** Stop in open loop. */
  public default void stop() {}
  /** Gets the current draw from the implementation type */
  	/**
	 * Get a list of the SelfChecking interface for all hardware in that
	 * implementation
	 */
	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}

}
