package frc.robot.subsystems.drive;

// import
// com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAnalogSensor;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAnalogSensor.Mode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;

// import
// edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.drive.DriveConstants;

public class SwerveModule extends SubsystemBase {
	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;
	private final RelativeEncoder driveEncoder;
	private final RelativeEncoder turningEncoder;
	private final PIDController turningPIDController;
	private final PIDController drivePIDController; //not using profiled cuz no angles
	//private SimpleMotorFeedforward turningFeedForward = null;
	private SimpleMotorFeedforward driveFeedForward = null;
	private final SparkAnalogSensor absoluteEncoder;
	//private final AnalogInput absoluteEncoder; // Use either AnalogInput or CANCoder depending on the absolute encoder
	//private final CANCoder absoluteEncoder;
	private final boolean absoluteEncoderReversed;
	private final double absoluteEncoderOffsetRad;
	//Simulation Variables
	double m_currentAngle;
	private double m_simDriveEncoderPosition;
	private double m_simDriveEncoderVelocity;
	private double m_simAngleDifference;
	private double m_simTurnAngleIncrement;
	Pose2d m_pose;
	private int m_moduleNumber;

	/**
	 * @param driveMotorId            Drive CANSparkMax Motor ID
	 * @param turningMotorId          Turning CANSparkMax Motor ID
	 * @param driveMotorReversed      True if Motor is Reversed
	 * @param turningMotorReversed    True if Motor is Reversed
	 * @param absoluteEncoderId       Turning Absolute Encoder ID
	 * @param absoluteEncoderOffset   Offset of Absolute Encoder in Radians
	 * @param absoluteEncoderReversed True if Encoder is Reversed
	 */
	public SwerveModule(int driveMotorId, int turningMotorId,
			boolean driveMotorReversed, boolean turningMotorReversed,
			int absoluteEncoderId, double absoluteEncoderOffset,
			boolean absoluteEncoderReversed, double[] driveKpKsKvKa,
			double[] turningKpKsKvKa) {
		/*turningFeedForward = new SimpleMotorFeedforward(
		 turningKpKsKvKa[1], turningKpKsKvKa[2], turningKpKsKvKa[3]);*/
		driveFeedForward = new SimpleMotorFeedforward(driveKpKsKvKa[1],
				driveKpKsKvKa[2], driveKpKsKvKa[3]);
		if (turningMotorId == 17) {
			m_moduleNumber = 0; //frontLeft
		} else if (turningMotorId == 11) {
			m_moduleNumber = 1; //frontRight
		} else if (turningMotorId == 15) {
			m_moduleNumber = 2; //backLeft
		} else if (turningMotorId == 13) {
			m_moduleNumber = 3; //backRight
		}
		//sets values of the encoder offset and whether its reversed
		this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
		this.absoluteEncoderReversed = absoluteEncoderReversed;
		//absoluteEncoder = new AnalogInput(absoluteEncoderId);
		//absoluteEncoder = new CANCoder(absoluteEncoderId);
		//declares motors
		driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
		turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
		//checks to see if they're inverted
		driveMotor.setInverted(driveMotorReversed);
		turningMotor.setInverted(turningMotorReversed);
		//sets the absolute encoder value (called this way because we have breakout boards in the motors)
		absoluteEncoder = turningMotor.getAnalog(Mode.kAbsolute);
		//relative encoder declarations
		driveEncoder = driveMotor.getEncoder();
		turningEncoder = turningMotor.getEncoder();
		//sets motor idle modes to break
		driveMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setIdleMode(IdleMode.kBrake);
		//accounts for gear ratios
		driveEncoder.setPositionConversionFactor(
				DriveConstants.SwerveConstants.kDriveEncoderRot2Meter);
		driveEncoder.setVelocityConversionFactor(
				DriveConstants.SwerveConstants.kDriveEncoderRPM2MeterPerSec);
		turningEncoder.setPositionConversionFactor(
				DriveConstants.SwerveConstants.kTurningEncoderRot2Rad);
		turningEncoder.setVelocityConversionFactor(
				DriveConstants.SwerveConstants.kTurningEncoderRPM2RadPerSec);
		//creates pidController, used exclusively for turning because that has to be precise
		turningPIDController = new PIDController(.5, 0, 0);
		//makes the value loop around
		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		drivePIDController = new PIDController(driveKpKsKvKa[0], 0, 0);
		if (Constants.currentMode == Constants.Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
			REVPhysicsSim.getInstance().addSparkMax(turningMotor,
					DCMotor.getNEO(1));
		}
	}

	public double getDrivePosition() {
		//returns the position of the drive wheel
		if (Constants.currentMode == Constants.Mode.REAL) {
			return driveEncoder.getPosition();
		} else {
			return m_simDriveEncoderPosition;
		}
	}

	public double getTurningPosition() {
		//returns the heading of the swerve module (turning motor position)
		if (Constants.currentMode == Constants.Mode.REAL) {
			return getAbsoluteEncoderRad();
		} else {
			return m_currentAngle;
		}
	}

	public int getModuleNumber() { return m_moduleNumber; }

	public Rotation2d getHeadingRotation2d() {
		return Rotation2d.fromDegrees(getTurningPosition());
	}

	public double getDriveVelocity() {
		//returns velocity of drive wheel
		if (Constants.currentMode == Constants.Mode.REAL) {
			return driveEncoder.getVelocity();
		} else {
			return m_simDriveEncoderVelocity;
		}
	}

	public double getTurningVelocity() {
		//returns velocity of turning motor
		return turningEncoder.getVelocity();
	}

	public void setTurningTest(double volts) { turningMotor.setVoltage(volts); }

	public void setDriveTest(double volts) {
		double turnOutput = turningPIDController.calculate(getAbsoluteEncoderRad(), 0);
		turningMotor.setVoltage(turnOutput);
		driveMotor.setVoltage(volts);
	}

	public double getAbsoluteEncoderRad() {
		//gets the voltage and divides by the maximum voltage to get a percent, then multiplies that percent by 2pi to get a degree heading.
		double angle = absoluteEncoder.getVoltage()
				/ RobotController.getVoltage3V3(); // use 5V when plugged into RIO 3.3V when using breakout board
		angle *= 2 * Math.PI;
		//adds the offset in
		angle -= absoluteEncoderOffsetRad;
		/*this line of code is here because our pid loop is set to negative pi to pi,
		but our absolute encoders read from 0 to 2pi. 
		The line of code below basically says to add 2pi if an input is below 0 to get it in the range of 0 to 2pi*/
		angle += angle <= 0 ? 2 * Math.PI : 0;
		//subtracts pi to make the 0 to 2pi range back into pi to -pi
		angle -= Math.PI; //angle > Math.PI ? 2*Math.PI : 0;
		//if encoder is reversed multiply the input by negative one
		angle *= (absoluteEncoderReversed ? -1 : 1);
		return angle;
	}
	
	public void resetEncoders() {
		//resets the encoders, (drive motor becomes zero, turning encoder becomes the module heading from the absolute encoder)
		driveEncoder.setPosition(0);
		turningEncoder.setPosition(getAbsoluteEncoderRad());
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
			driveMotor.setVoltage(driveOutput + driveFeedforward);
			turningMotor.set(turnOutput);
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

	@SuppressWarnings("unused") //incase we want accurate sim turn (this doesn't work right now tho!)
	private void simTurnPosition(double angle) {
		if (angle != m_currentAngle && m_simTurnAngleIncrement == 0) {
			m_simAngleDifference = angle - m_currentAngle;
			m_simTurnAngleIncrement = m_simAngleDifference * .02;// 10*50ms = .2 sec move time
		}
		if (m_simTurnAngleIncrement != 0) {
			m_currentAngle += m_simTurnAngleIncrement;
			if ((Math.abs(angle - m_currentAngle)) < .1) {
				m_currentAngle = angle;
				m_simTurnAngleIncrement = 0;
			}
		}
	}

	public void stop() {
		driveMotor.set(0);
		turningMotor.set(0);
	}

	public void setModulePose(Pose2d pose) { m_pose = pose; }

	public Pose2d getModulePose() { return m_pose; }

	@Override
	public void simulationPeriodic() { REVPhysicsSim.getInstance().run(); }
}
