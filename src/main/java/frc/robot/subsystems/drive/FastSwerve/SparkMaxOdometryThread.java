package frc.robot.subsystems.drive.FastSwerve;

import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 * <p>
 * This version is intended for devices like the SparkMax that require polling
 * rather than a blocking thread. A Notifier thread is used to gather samples
 * with consistent timing.
 */
public class SparkMaxOdometryThread {
	private List<DoubleSupplier> signals = new ArrayList<>();
	private List<Queue<Double>> queues = new ArrayList<>();
	private final Notifier notifier;
	private static SparkMaxOdometryThread instance = null;

	public static SparkMaxOdometryThread getInstance() {
		if (instance == null) {
			instance = new SparkMaxOdometryThread();
		}
		return instance;
	}

	private SparkMaxOdometryThread() {
		notifier = new Notifier(this::periodic);
		notifier.setName("SparkMaxOdometryThread");
		notifier.startPeriodic(1.0 / 250);
	}

	public Queue<Double> registerSignal(DoubleSupplier signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		Swerve.odometryLock.lock();
		try {
			signals.add(signal);
			queues.add(queue);
		}
		finally {
			Swerve.odometryLock.unlock();
		}
		return queue;
	}

	private void periodic() {
		Swerve.odometryLock.lock();
		Swerve.timestampQueue.offer(Logger.getRealTimestamp() / 1.0e6);
		try {
			for (int i = 0; i < signals.size(); i++) {
				queues.get(i).offer(signals.get(i).getAsDouble());
			}
		}
		finally {
			Swerve.odometryLock.unlock();
		}
	}
}
