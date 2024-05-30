package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public interface DrivetrainS extends Subsystem {
	/**
	 * Make the chassis go at a set speed
	 * 
	 * @param speeds the speed to set it to
	 */
	void setChassisSpeeds(ChassisSpeeds speeds);

	/**
	 * @return the ChassisSpeeds of the drivetrain
	 */
	ChassisSpeeds getChassisSpeeds();

	/**
	 * Reset the drivetrain's odometry to a particular pose
	 * 
	 * @param pose the pose to set it to
	 */
	void resetPose(Pose2d pose);

	/**
	 * Apply a particular vision measurement to the drivetrain
	 * 
	 * @param pose       the pose returned by the vision estimate
	 * @param timestamp  the timestamp of the pose
	 * @param estStdDevs the estimated std dev (pose's difference from the mean
	 *                      in x, y, and theta)
	 */
	void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs);

	/**
	 * @return the pose of the robot
	 */
	Pose2d getPose();

	/**
	 * Stops the drivetrain
	 */
	void stopModules();

	/**
	 * @return the angle of the drivetrain as a rotation2d (in degrees)
	 */
	Rotation2d getRotation2d();

	/**
	 * SysID Command for drivetrain characterization
	 * 
	 * @param kreverse the direction
	 * @return a command executing the characterization step
	 */
	Command sysIdDynamicTurn(Direction kreverse);

	/**
	 * SysID Command for drivetrain characterization
	 * 
	 * @param kreverse the direction
	 * @return a command executing the characterization step
	 */
	Command sysIdQuasistaticTurn(Direction kforwards);

	/**
	 * SysID Command for drivetrain characterization
	 * 
	 * @param kforward the direction
	 * @return a command executing the characterization step
	 */
	Command sysIdDynamicDrive(Direction kforward);

	/**
	 * SysID Command for drivetrain characterization
	 * 
	 * @param kforward the direction
	 * @return a command executing the characterization step
	 */
	Command sysIdQuasistaticDrive(Direction kreverse);

	/**
	 * Reset the heading of the drivetrain
	 */
	void zeroHeading();

	/**
	 * @return if the gyro drivetrain is connected
	 */
	boolean isConnected();

	/**
	 * CTRE Nonsense
	 */
	default void applyRequest() {
		throw new UnsupportedOperationException("No support for requests.");
	}
}
