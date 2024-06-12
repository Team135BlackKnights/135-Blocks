package frc.robot.subsystems.drive.REVSwerve.SwerveModules;

// import
// com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
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
import frc.robot.utils.drive.DriveConstants.TrainConstants;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;

// import
// edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.REVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class REVSwerveModule extends SubsystemBase {
	private static ModulePosition position;
	private CANSparkBase driveMotor, turningMotor;
	private RelativeEncoder driveEncoder, turningEncoder;
	private DCMotorSim driveMotorSim = null, turningMotorSim = null;
	private double absoluteEncoderOffsetRad;
	private boolean absoluteEncoderReversed;
	private SparkAnalogSensor absoluteEncoder;
	private PIDController turningPIDController = null, drivePIDController = null;
	private SimpleMotorFeedforward driveFeedForward = null;
	private double m_currentAngle = 0, m_simDriveEncoderPosition = 0,
			m_simDriveEncoderVelocity = 0, m_simAngleDifference = 0,
			m_simTurnAngleIncrement = 0, m_moduleMaxSpeed = 0;
	private Pose2d m_pose = new Pose2d();
	private int m_moduleNumber = 0;

	//private final AnalogInput absoluteEncoder; // Use either AnalogInput or CANCoder depending on the absolute encoder
	//private final CANCoder absoluteEncoder;
	/**
	 * @param driveMotorId            Drive CANSparkFlex Motor ID
	 * @param turningMotorId          Turning CANSparkFlex Motor ID
	 * @param driveMotorReversed      True if Motor is Reversed
	 * @param turningMotorReversed    True if Motor is Reversed
	 * @param absoluteEncoderOffset   Offset of Absolute Encoder in Radians
	 * @param absoluteEncoderReversed True if Encoder is Reversed
	 */
	public REVSwerveModule(REVModuleConstantContainer container) {
		position = container.getModulePosition();
		switch (position) {
		case FRONT_LEFT:
			this.m_moduleNumber = 0;
			break;
		case FRONT_RIGHT:
			this.m_moduleNumber = 1;
			break;
		case BACK_LEFT:
			this.m_moduleNumber = 2;
			break;
		case BACK_RIGHT:
			this.m_moduleNumber = 3;
			break;
		default:
			this.m_moduleNumber = -1;
			break;
		}
		/*turningFeedForward = new SimpleMotorFeedforward(
		 turningKpKsKvKa[1], turningKpKsKvKa[2], turningKpKsKvKa[3]);*/
		driveFeedForward = new SimpleMotorFeedforward(
				container.getDriveMotorConstantContainer().getKs(),
				container.getDriveMotorConstantContainer().getKv(),
				container.getDriveMotorConstantContainer().getKa());
		//sets values of the encoder offset and whether its reversed
		absoluteEncoderOffsetRad = container.getAbsoluteEncoderOffset();
		this.absoluteEncoderReversed = container.getAbsoluteEncoderReversed();
		m_moduleMaxSpeed = container.getModuleMaxSpeed();
		//absoluteEncoder = new AnalogInput(absoluteEncoderId);
		//absoluteEncoder = new CANCoder(absoluteEncoderId);
		//declares motors

		switch (DriveConstants.robotMotorController) {
			
		case NEO_SPARK_MAX:
			System.err.println("Detected Spark Max");
			driveMotor = new CANSparkMax(container.getDriveMotorID(),
					MotorType.kBrushless);
			turningMotor = new CANSparkMax(container.getTurningMotorID(),
					MotorType.kBrushless); 
			switch (Constants.currentMode) {
				case SIM:
					driveMotorSim = new DCMotorSim(DCMotor.getNEO(1), TrainConstants.kDriveMotorGearRatio, .0000001);
					turningMotorSim = new DCMotorSim(DCMotor.getNEO(1),TrainConstants.kTurningMotorGearRatio,.0000001);
				break;
			default:
				break;
			}
		case VORTEX_SPARK_FLEX:
			System.err.println("Detected Spark Flex");
			driveMotor = new CANSparkFlex(container.getDriveMotorID(),
					MotorType.kBrushless);
			turningMotor = new CANSparkFlex(container.getTurningMotorID(),
					MotorType.kBrushless);
			switch (Constants.currentMode) {
				case SIM:
					driveMotorSim = new DCMotorSim(DCMotor.getNeoVortex(1), TrainConstants.kDriveMotorGearRatio, .0000001);
					turningMotorSim = new DCMotorSim(DCMotor.getNeoVortex(1),TrainConstants.kTurningMotorGearRatio,.0000001);
				break;
			default:
				break;
			}
		default:
			break;
		}
		//checks to see if they're inverted
		driveMotor.setInverted(container.getDriveMotorReversed());
		turningMotor.setInverted(container.getTurningMotorReversed());
		//sets the absolute encoder value (called way because we have breakout boards in the motors)
		absoluteEncoder = turningMotor.getAnalog(Mode.kAbsolute);
		//relative encoder declarations
		driveEncoder = driveMotor.getEncoder();
		turningEncoder = turningMotor.getEncoder();
		//sets motor idle modes to break
		driveMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setIdleMode(IdleMode.kBrake);
		//accounts for gear ratios
		driveEncoder.setPositionConversionFactor(container
				.getSwerveModuleEncoderConstants().getDriveEncoderRot2Meter());
		driveEncoder.setVelocityConversionFactor(
				container.getSwerveModuleEncoderConstants()
						.getDriveEncoderRPM2MeterPerSec());
		turningEncoder.setPositionConversionFactor(container
				.getSwerveModuleEncoderConstants().getTurningEncoderRot2Rad());
		turningEncoder.setVelocityConversionFactor(
				container.getSwerveModuleEncoderConstants()
						.getTurningEncoderRPM2RadPerSec());
		//creates pidController, used exclusively for turning because that has to be precise
		turningPIDController = new PIDController(.5, 0, 0);
		//makes the value loop around
		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		drivePIDController = new PIDController(
				container.getDriveMotorConstantContainer().getP(), 0,
				container.getDriveMotorConstantContainer().getD());
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
		double turnOutput = turningPIDController
				.calculate(getAbsoluteEncoderRad(), 0);
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
		/*line of code is here because our pid loop is set to negative pi to pi,
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

	@SuppressWarnings("unused") //incase we want accurate sim turn (doesn't work right now tho!)
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

	/**
	 * Converts the inputs from meters to volts, sets motors
	 * 
	 * @param driveOutput      driveOutput, in METERS
	 * @param driveFeedforward driveFeedForwards, in Meters
	 * @param turnOutput       Nish add details
	 */
	public void setMotors(double driveOutput, double driveFeedforward,
			double turnOutput) {
		double volts = 12 * (driveOutput + driveFeedforward) / m_moduleMaxSpeed;
		SmartDashboard.putNumber("Output", volts);
		driveMotor.setVoltage(volts);
		turningMotor.set(turnOutput);
	}

	public int getModuleNumber() { return m_moduleNumber; }

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
		double driveOutput = drivePIDController.calculate(getDriveVelocity(),
				state.speedMetersPerSecond);
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
