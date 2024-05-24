package frc.robot.subsystems.drive.SwerveModules;

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
// import
// edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.MotorConstantContainer;

public class CANSparkMaxSwerveModule extends SwerveMotorControllers  {
	private CANSparkMax driveMotor;
	private CANSparkMax turningMotor;
	private RelativeEncoder driveEncoder;
	private RelativeEncoder turningEncoder;

	private double absoluteEncoderOffsetRad;
	private boolean absoluteEncoderReversed;
	private SparkAnalogSensor absoluteEncoder;
	//private final AnalogInput absoluteEncoder; // Use either AnalogInput or CANCoder depending on the absolute encoder
	//private final CANCoder absoluteEncoder;


	public CANSparkMaxSwerveModule(int driveMotorId, int turningMotorId,
	boolean driveMotorReversed, boolean turningMotorReversed,
	int absoluteEncoderId, double absoluteEncoderOffset,
	boolean absoluteEncoderReversed, MotorConstantContainer driveMotorConstantContainer,
	MotorConstantContainer turningKpKsKvKa) { 
		initialize(driveMotorId, turningMotorId, driveMotorReversed, turningMotorReversed, absoluteEncoderId, absoluteEncoderOffset, absoluteEncoderReversed, driveMotorConstantContainer, turningKpKsKvKa);
	}
	/**Sim still needs some way to be implemented (maybe use a wrapper?)
	 * This is literally the CANSparkMax swerve module code but I did CTRL+F and replaced CANSparkFlex with CANSparkMax -Nish
	 * 
	 * @param driveMotorId            Drive CANSparkFlex Motor ID
	 * @param turningMotorId          Turning CANSparkFlex Motor ID
	 * @param driveMotorReversed      True if Motor is Reversed
	 * @param turningMotorReversed    True if Motor is Reversed
	 * @param absoluteEncoderId       Turning Absolute Encoder ID
	 * @param absoluteEncoderOffset   Offset of Absolute Encoder in Radians
	 * @param absoluteEncoderReversed True if Encoder is Reversed
	 */
	
	@Override
	public void initialize(int driveMotorId, int turningMotorId,
			boolean driveMotorReversed, boolean turningMotorReversed,
			int absoluteEncoderId, double absoluteEncoderOffset,
			boolean absoluteEncoderReversed, MotorConstantContainer driveMotorConstantContainer,
			MotorConstantContainer turningKpKsKvKa) {
		/*turningFeedForward = new SimpleMotorFeedforward(
		 turningKpKsKvKa[1], turningKpKsKvKa[2], turningKpKsKvKa[3]);*/
		driveFeedForward = new SimpleMotorFeedforward(driveMotorConstantContainer.getKs(),
				driveMotorConstantContainer.getKv(), driveMotorConstantContainer.getKa());
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
		drivePIDController = new PIDController(driveMotorConstantContainer.getP(), 0, driveMotorConstantContainer.getD());
		if (Constants.currentMode == Constants.Mode.SIM) {
			//TODO:Figure out simulation for this
			/*
			 * REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
				REVPhysicsSim.getInstance().addSparkMax(turningMotor, DCMotor.getNEO(1));
			 */

		}
	}
	@Override
	public double getDrivePosition() {
		//returns the position of the drive wheel
		if (Constants.currentMode == Constants.Mode.REAL) {
			return driveEncoder.getPosition();
		} else {
			return m_simDriveEncoderPosition;
		}
	}
	@Override
	public double getTurningPosition() {
		//returns the heading of the swerve module (turning motor position)
		if (Constants.currentMode == Constants.Mode.REAL) {
			return getAbsoluteEncoderRad();
		} else {
			return m_currentAngle;
		}
	}
	@Override
	public double getDriveVelocity() {
		//returns velocity of drive wheel
		if (Constants.currentMode == Constants.Mode.REAL) {
			return driveEncoder.getVelocity();
		} else {
			return m_simDriveEncoderVelocity;
		}
	}
	@Override
	public double getTurningVelocity() {
		//returns velocity of turning motor
		return turningEncoder.getVelocity();
	}
	@Override
	public void setTurningTest(double volts) { turningMotor.setVoltage(volts); }
	@Override
	public void setDriveTest(double volts) {
		double turnOutput = turningPIDController.calculate(getAbsoluteEncoderRad(), 0);
		turningMotor.setVoltage(turnOutput);
		driveMotor.setVoltage(volts);
	}
	@Override
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
	@Override
	public void resetEncoders() {
		//resets the encoders, (drive motor becomes zero, turning encoder becomes the module heading from the absolute encoder)
		driveEncoder.setPosition(0);
		turningEncoder.setPosition(getAbsoluteEncoderRad());
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
	@Override
	public void stop() {
		driveMotor.set(0);
		turningMotor.set(0);
	}
	@Override
	public void setMotors(double driveOutput, double driveFeedforward,
			double turnOutput) {
				driveMotor.setVoltage(driveOutput + driveFeedforward);
				turningMotor.set(turnOutput);
	}
}

