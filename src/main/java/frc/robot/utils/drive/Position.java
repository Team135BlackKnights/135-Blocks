package frc.robot.utils.drive;

public class Position<T> {
	private final T positions;
	private final double timestamp;
	/**
	 * Create a new Position for the drivebase
	 * @param positions defaulted to ambiguous variable. 
	 * @param timestamp in Logger.getTimestamp()
	 */
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
