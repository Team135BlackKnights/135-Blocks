// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.utils.drive.Sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.FastSwerve.Module;
import frc.robot.subsystems.drive.FastSwerve.PhoenixOdometryThread;
import frc.robot.subsystems.drive.FastSwerve.SparkMaxOdometryThread;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingPigeon2;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
	private final Pigeon2 pigeon = new Pigeon2(30);
	private final StatusSignal<Double> yaw = pigeon.getYaw();
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;
	private final StatusSignal<Double> yawVelocity = pigeon
			.getAngularVelocityZWorld();
	private final StatusSignal<Double> accelX = pigeon.getAccelerationX();
	private final StatusSignal<Double> accelY = pigeon.getAccelerationY();
	private double last_world_linear_accel_x, last_world_linear_accel_y;

	public GyroIOPigeon2(boolean phoenixDrive) {
		pigeon.getConfigurator().apply(new Pigeon2Configuration());
		pigeon.getConfigurator().setYaw(0.0);
		yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);
		yawVelocity.setUpdateFrequency(100.0);
		pigeon.optimizeBusUtilization();
		if (phoenixDrive) {
			yawTimestampQueue = PhoenixOdometryThread.getInstance()
					.makeTimestampQueue();
			yawPositionQueue = PhoenixOdometryThread.getInstance()
					.registerSignal(pigeon, pigeon.getYaw());
		} else {
			yawTimestampQueue = SparkMaxOdometryThread.getInstance()
					.makeTimestampQueue();
			yawPositionQueue = SparkMaxOdometryThread.getInstance()
					.registerSignal(() -> {
						boolean valid = yaw.refresh().getStatus().isOK();
						if (valid) {
							return OptionalDouble.of(yaw.getValueAsDouble());
						} else {
							return OptionalDouble.empty();
						}
					});
		}
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingPigeon2("IMU", pigeon));
		return hardware;
	}

	@Override
	public void reset() { pigeon.reset(); }

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = BaseStatusSignal
				.refreshAll(yaw, yawVelocity, accelX, accelY).equals(StatusCode.OK);
		inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
		inputs.yawVelocityRadPerSec = Units
				.degreesToRadians(yawVelocity.getValueAsDouble());
		inputs.odometryYawTimestamps = yawTimestampQueue.stream()
				.mapToDouble((Double value) -> value).toArray();
		inputs.odometryYawPositions = yawPositionQueue.stream()
				.map((Double value) -> Rotation2d.fromDegrees(value))
				.toArray(Rotation2d[]::new);
		yawTimestampQueue.clear();
		yawPositionQueue.clear();
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
}
