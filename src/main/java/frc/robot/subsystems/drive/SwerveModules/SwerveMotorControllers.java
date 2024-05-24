package frc.robot.subsystems.drive.SwerveModules;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.utils.drive.MotorConstantContainer;

public abstract class SwerveMotorControllers {
	public static PIDController turningPIDController = null;
	public static PIDController drivePIDController = null;
	public static SimpleMotorFeedforward driveFeedForward = null;
	public static double m_currentAngle = 0;
	public static double m_simDriveEncoderPosition = 0;
	public static double m_simDriveEncoderVelocity = 0;
	public static double m_simAngleDifference = 0;
	public static double m_simTurnAngleIncrement = 0;
	public static Pose2d m_pose = new Pose2d();
	public static int m_moduleNumber = 0;

	public abstract void initialize(int driveMotorId, int turningMotorId,
			boolean driveMotorReversed, boolean turningMotorReversed,
			int absoluteEncoderId, double absoluteEncoderOffset,
			boolean absoluteEncoderReversed,
			MotorConstantContainer driveMotorConstantContainer,
			MotorConstantContainer turningKpKsKvKa);

	public abstract double getDrivePosition();

	public abstract double getTurningPosition();

	public abstract double getDriveVelocity();

	public abstract double getTurningVelocity();

	public abstract void setTurningTest(double volts);

	public abstract void setDriveTest(double volts);

	public abstract double getAbsoluteEncoderRad();

	public abstract void resetEncoders();

	public int getModuleNumber(){
		return m_moduleNumber;
	}

	public abstract void stop();

	public abstract void setMotors(double driveOutput, double driveFeedforward,
			double turnOutput);

	public Rotation2d getHeadingRotation2d() {
		return Rotation2d.fromDegrees(getTurningPosition());
	}

	public SwerveModulePosition getPosition() {
		return new SwerveModulePosition(getDrivePosition(),
				getHeadingRotation2d());
		//basically creates a new swervemoduleposition based on the current positions of the drive and turning encoders
	}

	public SwerveModuleState getState() {
		//creates new swerveModuleState based on drive speed and turn motor position (speed and direction)
		return new SwerveModuleState(getDriveVelocity(), getHeadingRotation2d());
	}

	public void setDesiredState(SwerveModuleState state) {
		//var encoderRotation = new Rotation2d(getTurningPosition());
		// Stops the motors if the desired state is too small
		/* if (Math.abs(state.speedMetersPerSecond) < 0.001 && !SwerveS.autoLock) {
		    stop();
		    return;
		}*/
		// Optimizing finds the shortest path to the desired angle
		state = SwerveModuleState.optimize(state, getState().angle);
		// Calculate the drive output from the drive PID controller.
		final double driveOutput = drivePIDController
				.calculate(getDriveVelocity(), state.speedMetersPerSecond);
		final double driveFeedforward = driveFeedForward
				.calculate(state.speedMetersPerSecond);
		// Calculate the turning motor output from the turning PID controller.
		final double turnOutput = turningPIDController
				.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());
		//        final double turnFeedforward =
		//         turningFeedForward.calculate(turningPIDController.getSetpoint().velocity);
		if (Constants.currentMode == Constants.Mode.REAL) {
			setMotors(driveOutput, driveFeedforward, turnOutput);
		} else {
			simUpdateDrivePosition(state);
			m_currentAngle = state.angle.getDegrees();
			//simTurnPosition(m_currentAngle);
		}
	}

	private void simUpdateDrivePosition(SwerveModuleState state) {
		m_simDriveEncoderVelocity = state.speedMetersPerSecond;
		double distancePer20Ms = m_simDriveEncoderVelocity * .02;
		m_simDriveEncoderPosition += distancePer20Ms;
	}

	public void setModulePose(Pose2d pose) { m_pose = pose; }

	public Pose2d getModulePose() { return m_pose; }
}
