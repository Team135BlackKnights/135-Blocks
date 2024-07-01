package frc.robot.utils.drive.Sensors;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.utils.selfCheck.SelfChecking;

public interface DistanceSensorIO {
	@AutoLog
	public static class DistanceSensorIOInputs {
		public double distanceMeters = 0.0;
		public double ambientLightLevel = 0.0;
		public int statusCode = -1;
	}

	public default void updateInputs(DistanceSensorIOInputs inputs) {}

	public default List<SelfChecking> getSelfCheckingHardware() {
		return new ArrayList<SelfChecking>();
	}
}
