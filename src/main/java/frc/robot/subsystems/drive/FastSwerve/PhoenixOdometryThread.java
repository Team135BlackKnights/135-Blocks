package frc.robot.subsystems.drive.FastSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.ParentDevice;

import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.DriveConstants.GyroType;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 * <p>
 * This version is intended for Phoenix 6 devices on both the RIO and CANivore
 * buses. When using a CANivore, the thread uses the "waitForAll" blocking
 * method to enable more consistent sampling. This also allows Phoenix Pro users
 * to benefit from lower latency between devices using CANivore time
 * synchronization.
 */
public class PhoenixOdometryThread extends Thread {
	private final Lock signalsLock = new ReentrantLock(); // Prevents conflicts when registering signals
	private BaseStatusSignal[] signals = new BaseStatusSignal[0];
	private List<Supplier<OptionalDouble>> navXSignals;
	private final List<Queue<Double>> queues = new ArrayList<>();
	private final List<Queue<Double>> timestampQueues = new ArrayList<>();
	private boolean isCANFD = false;
	private static PhoenixOdometryThread instance = null;

	public static PhoenixOdometryThread getInstance() {
		if (instance == null) {
			instance = new PhoenixOdometryThread();
		}
		return instance;
	}

	private PhoenixOdometryThread() {
		setName("PhoenixOdometryThread");
		setDaemon(true);
		if (DriveConstants.gyroType == GyroType.NAVX){
			navXSignals = new ArrayList<>();
		}
	}

	@Override
	public void start() {
		if (timestampQueues.size() > 0) {
			super.start();
		}
	}

	public Queue<Double> registerSignal(ParentDevice device,
			StatusSignal<Double> signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		signalsLock.lock();
		Swerve.odometryLock.lock();
		try {
			isCANFD = CANBus.isNetworkFD(device.getNetwork());
			BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length
					+ 1];
			System.arraycopy(signals, 0, newSignals, 0, signals.length);
			newSignals[signals.length] = signal;
			signals = newSignals;
			queues.add(queue);
		}
		finally {
			signalsLock.unlock();
			Swerve.odometryLock.unlock();
		}
		return queue;
	}
	public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		Swerve.odometryLock.lock();
		try {
			navXSignals.add(signal);
			queues.add(queue);
		}
		finally {
			Swerve.odometryLock.unlock();
		}
		return queue;
	}

	public Queue<Double> makeTimestampQueue() {
		Queue<Double> queue = new ArrayBlockingQueue<>(20);
		Swerve.odometryLock.lock();
		try {
			timestampQueues.add(queue);
		}
		finally {
			Swerve.odometryLock.unlock();
		}
		return queue;
	}

	@Override
	public void run() {
		while (true) {
			// Wait for updates from all signals
			signalsLock.lock();
			try {
				if (isCANFD) {
					BaseStatusSignal.waitForAll(2.0 / Module.ODOMETRY_FREQUENCY,
							signals);
				} else {
					// "waitForAll" does not support blocking on multiple
					// signals with a bus that is not CAN FD, regardless
					// of Pro licensing. No reasoning for this behavior
					// is provided by the documentation.
					Thread.sleep((long) (1000.0 / Module.ODOMETRY_FREQUENCY));
					if (signals.length > 0)
						BaseStatusSignal.refreshAll(signals);
				}
			}
			catch (InterruptedException e) {
				e.printStackTrace();
			}
			finally {
				signalsLock.unlock();
			}
			// Save new data to queues
			Swerve.odometryLock.lock();
			try {
				double timestamp = Logger.getRealTimestamp() / 1e6;
				double totalLatency = 0.0;
				for (BaseStatusSignal signal : signals) {
					totalLatency += signal.getTimestamp().getLatency();
				}
				if (signals.length > 0) {
					timestamp -= totalLatency / signals.length;
				}
				for (int i = 0; i < signals.length; i++) {
					queues.get(i).offer(signals[i].getValueAsDouble());
				}
				for (int i = 0; i < timestampQueues.size(); i++) {
					timestampQueues.get(i).offer(timestamp);
				}
			}
			finally {
				Swerve.odometryLock.unlock();
			}
		}
	}
}
