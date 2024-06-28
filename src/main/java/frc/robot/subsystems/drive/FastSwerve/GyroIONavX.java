// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.FastSwerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SelfCheckingNavX2;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;



/** IO implementation for navX
 * @apiNote Needs to be tested
 */
public class GyroIONavX implements GyroIO {
	private final AHRS navX = new AHRS(Port.kUSB);
	private final Queue<Double> yawPositionQueue;
	private final Queue<Double> yawTimestampQueue;
	private double last_world_linear_accel_x, last_world_linear_accel_y, current_angle_position, last_angle_position = 0;

	public GyroIONavX(boolean phoenixDrive) {
		if (phoenixDrive) {
			yawTimestampQueue = PhoenixOdometryThread.getInstance()
					.makeTimestampQueue();
			yawPositionQueue = PhoenixOdometryThread.getInstance()
					.registerSignal(() -> {
						boolean valid = navX.isConnected();
						if (valid) {
							return OptionalDouble.of(navX.getAngle());
						} else {
							return OptionalDouble.empty();
						}
					});
		} else {
			yawTimestampQueue = SparkMaxOdometryThread.getInstance()
					.makeTimestampQueue();
			yawPositionQueue = SparkMaxOdometryThread.getInstance()
					.registerSignal(() -> {
						boolean valid = navX.isConnected();
						if (valid) {
							return OptionalDouble.of(navX.getAngle());
						} else {
							return OptionalDouble.empty();
						}
					});
		}
	}

	@Override
	public List<SelfChecking> getSelfCheckingHardware() {
		List<SelfChecking> hardware = new ArrayList<SelfChecking>();
		hardware.add(new SelfCheckingNavX2("IMU", navX));
		return hardware;
	}

	@Override
	public void reset() { navX.reset(); }

	@Override
	public void updateInputs(GyroIOInputs inputs) {
		inputs.connected = navX.isConnected();
		current_angle_position = navX.getAngle();
		inputs.yawPosition = Rotation2d.fromDegrees(current_angle_position);
		inputs.yawVelocityRadPerSec = Units
				.degreesToRadians((current_angle_position-last_angle_position)/Module.ODOMETRY_FREQUENCY);
		last_angle_position = current_angle_position;
		inputs.odometryYawTimestamps = yawTimestampQueue.stream()
				.mapToDouble((Double value) -> value).toArray();
		inputs.odometryYawPositions = yawPositionQueue.stream()
				.map((Double value) -> Rotation2d.fromDegrees(value))
				.toArray(Rotation2d[]::new);
		yawTimestampQueue.clear();
		yawPositionQueue.clear();
		double curr_world_linear_accel_x = navX.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = navX.getWorldLinearAccelY();
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
