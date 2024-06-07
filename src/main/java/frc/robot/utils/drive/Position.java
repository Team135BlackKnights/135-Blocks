package frc.robot.utils.drive;

public class Position<T> {
	private final T positions;
	private final double timestamp;

	public Position(T positions, double timestamp) {
		 this.positions = positions;
		 this.timestamp = timestamp;
	}

	public T getPositions() {
		 return positions;
	}

	public double getTimestamp() {
		 return timestamp;
	}
}
