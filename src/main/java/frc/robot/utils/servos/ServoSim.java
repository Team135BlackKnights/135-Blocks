package frc.robot.utils.servos;


import edu.wpi.first.math.util.Units;

import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;

/**
 * Class to handle simulation of a servo
 */
public class ServoSim {
	/**
	 * Assigns the mode of the servo in simulation (in range is within a certain
	 * set of bounds, continuous means to continually rotate)
	 */
	


	private double[] bounds = { 0, 0
	};
	private double positionDegrees = 0, velocityDegreesPerSecond = 0, maxRPM = 0, setpoint = 0;
	SimServoMode simMode;


	public ServoSim(SimServoMode runMode, ServoType type,
			double gearing, double initialPositionDegrees) {
		simMode = runMode;
		positionDegrees = initialPositionDegrees;
		switch (type) {
		case REVSmartServo:
				maxRPM = 3;
			break;
		default:

			break;
		}
		//Momentum is obnoxiously low, shaft doesnt have that much inertia

	}

	/**
	 * Set the bounds of a simulated servo in Degrees
	 * 
	 * @param leftBound  The leftmost position the servo can go to (in Degrees)
	 * @param rightBound The rightmost position the servo can go to (in Degrees)
	 */
	public void setSimBounds(double leftBound, double rightBound) {
		bounds = new double[] { leftBound, rightBound
		};
		//Assumes servo starts at leftBound
		positionDegrees = leftBound;
	}

	/**
	 * Sets the servo to a certain percent. If it's continuous, sets a POSITION
	 * from 0 to 1, where 0 is maximum left and 1 is maximum right If it's in
	 * range, set it to a constant PERCENTAGE of the max velocity
	 * 
	 * @param percent the percent to set the servo to (-1 to 1) if the servo is
	 *                   continuous, set to a position from (0 to 1.0) if it is
	 *                   in range mode
	 */
	public void set(double percent) {
		//RIO can only output a max of 5 volts on a pwm

	}

	/**
	 * @return the angular velocity in Degrees per second
	 */
	public double getAngularVelocityDegreesPerSec() {
		return velocityDegreesPerSecond;
	}

	/**
	 * @return the angular position in Degrees
	 */
	public double getAngularPositionDegrees() { return positionDegrees; }

	/**
	 * Call this in a periodic to update the servo state
	 * 
	 * @param dtSeconds the time since this was last called (in periodic this
	 *                     should be .02)
	 */
	public void updateServoSim(double dtSeconds) {
		//This is just using kinematics formulas,
		//We calculate velocity from maxRPM times voltage
		//And position from that times the dt
		switch (simMode) {
		case INRANGE:
			if ((positionDegrees < bounds[0] && velocityDegreesPerSecond < 0) || (positionDegrees > bounds[1] && velocityDegreesPerSecond > 0)){


			}
		
		default:
			
	

			break;
		}

	}
}
