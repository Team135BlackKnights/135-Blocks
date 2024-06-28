package frc.robot.subsystems.drive.OldREVSwerve.SwerveModules;

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
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;

// import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.OldREVSwerve.REVModuleConstantContainer;
import frc.robot.utils.drive.DriveConstants;

public class REVSwerveModule extends SubsystemBase {
	private static ModulePosition position;
	public CANSparkBase driveMotor, turningMotor;
	private RelativeEncoder driveEncoder, turningEncoder;
	private DCMotorSim driveMotorSim, turningMotorSim;
	private double absoluteEncoderOffsetRad;
	private boolean absoluteEncoderReversed;
	private SparkAnalogSensor absoluteEncoder;
	private PIDController turningPIDController = null, drivePIDController = null;
	private SimpleMotorFeedforward driveFeedForward = null;
	private double driveEncoderRot2Meter, driveEncoderRPM2MeterPerSec,
			turningEncoderRot2Rad, turningEncoderRPM2RadPerSec;
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
		driveEncoderRot2Meter = container.getDriveMotorGearing()*Math.PI;
		driveEncoderRPM2MeterPerSec = container.getDriveMotorGearing()*Math.PI/60;
		turningEncoderRot2Rad = container.getTurnMotorGearing()*2*Math.PI;
		turningEncoderRPM2RadPerSec = container.getTurnMotorGearing()*2*Math.PI/60;
		/*turningFeedForward = new SimpleMotorFeedforward(
		 turningKpKsKvKa[1], turningKpKsKvKa[2], turningKpKsKvKa[3]);*/
		driveFeedForward = new SimpleMotorFeedforward(
				container.getDriveMotorConstantContainer().getKs(),
				container.getDriveMotorConstantContainer().getKv(),
				container.getDriveMotorConstantContainer().getKa());
		//sets values of the encoder offset and whether its reversed
		absoluteEncoderOffsetRad = container.getAbsoluteEncoderOffset();
		this.absoluteEncoderReversed = container.getAbsoluteEncoderReversed();
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
				driveMotorSim = new DCMotorSim(DCMotor.getNEO(1), driveEncoderRot2Meter, .001); //container.getSwerveModuleEncoderConstants().getDriveEncoderRot2Meter()
				turningMotorSim = new DCMotorSim(DCMotor.getNEO(1), container.getTurnMotorGearing(), .001);
				break;
			case VORTEX_SPARK_FLEX:
				System.err.println("Detected Spark Flex");
				driveMotor = new CANSparkFlex(container.getDriveMotorID(),
						MotorType.kBrushless);
				turningMotor = new CANSparkFlex(container.getTurningMotorID(),
						MotorType.kBrushless);
				driveMotorSim = new DCMotorSim(DCMotor.getNeoVortex(1),
				driveEncoderRot2Meter, .001);
				turningMotorSim = new DCMotorSim(DCMotor.getNeoVortex(1),
				container.getTurnMotorGearing(), .001);
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
			driveEncoder.setPositionConversionFactor(driveEncoderRot2Meter);
			driveEncoder.setVelocityConversionFactor(driveEncoderRPM2MeterPerSec);
			turningEncoder.setPositionConversionFactor(turningEncoderRot2Rad);
			turningEncoder
					.setVelocityConversionFactor(turningEncoderRPM2RadPerSec);
			//creates pidController, used exclusively for turning because that has to be precise
		turningPIDController = new PIDController(360, 0, 0);
		//makes the value loop around
		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		drivePIDController = new PIDController(
				container.getDriveMotorConstantContainer().getP(), 0,
				container.getDriveMotorConstantContainer().getD());
	}

	public double getDrivePosition() {
			return driveEncoder.getPosition();
	}

	public double getTurningPosition() {
			return getAbsoluteEncoderRad();
	}

	public double getDriveVelocity() {
			if (Constants.currentMode == Constants.Mode.REAL){
				return driveEncoder.getVelocity();
			}
			return driveMotorSim.getAngularVelocityRPM()/60;
	}

	public double getTurningVelocity() {
			if (Constants.currentMode == Constants.Mode.REAL){
				return turningEncoder.getVelocity();
			}
			return turningMotorSim.getAngularVelocityRPM()/60;
		//returns velocity of turning motor
	}

	public void setTurningTest(double volts) { turningMotor.setVoltage(volts); }

	public void setDriveTest(double volts) {
		double turnOutput = turningPIDController
				.calculate(getAbsoluteEncoderRad(), 0);
		turningMotor.setVoltage(turnOutput);
		driveMotor.setVoltage(volts);
	}

	public double getAbsoluteEncoderRad() {
		switch (Constants.currentMode) {
		case REAL:
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
		default:
			//Sim encoder is 100 percent accurate, so we can just recycle that value
			return Units.degreesToRadians(turningEncoder.getPosition());
		}
	}

	/**
	 * Resets the encoders, (drive motor becomes zero, turning encoder becomes
	 * the module heading from the absolute encoder)
	 */
	public void resetEncoders() {
			driveEncoder.setPosition(0);
			turningEncoder.setPosition(getAbsoluteEncoderRad());
	}

	public void stop() {
		switch (Constants.currentMode) {
		case REAL:
			driveMotor.set(0);
			turningMotor.set(0);
			break;
		default:
			driveMotorSim.setInputVoltage(0);
			turningMotorSim.setInputVoltage(0);
			break;
		}
	}

	/**
	 * Converts the inputs from meters to volts, sets motors
	 * 
	 * @param driveOutput      driveOutput, in METERS
	 * @param driveFeedforward driveFeedForwards, in Meters
	 * @param turnOutput       turningOutput, as a percentage 
	 */
	public void setMotors(double driveOutput,
			double turnOutput) {
		switch (Constants.currentMode) {
		case REAL:
			driveMotor.setVoltage(driveOutput);
			turningMotor.set(turnOutput);
			break;
		default:
			driveMotorSim.setInputVoltage(driveOutput);
			turningMotorSim.setInputVoltage(turnOutput*12);
			break;
		}
	}

	public int getModuleNumber() { return m_moduleNumber; }

	public Rotation2d getHeadingRotation2d() {
		return Rotation2d.fromRadians(getTurningPosition());
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
	/**
	 * Updates the module states in sim. DT is set to .02, so call this in periodic or tweak this
	 */
	public void updateModuleStates(){
		if (Constants.currentMode == Constants.Mode.SIM){
			driveMotorSim.update(.02);
			turningMotorSim.update(.02);
			driveEncoder.setPosition(driveMotorSim.getAngularPositionRotations());
			turningEncoder.setPosition(turningMotorSim.getAngularPositionRotations());
		}

	}

	public void setDesiredState(SwerveModuleState state) {

		// Optimizing finds the shortest path to the desired angle
		state = SwerveModuleState.optimize(state, getState().angle);
		// Calculate the drive output from the drive PID controller.
		double driveOutput = drivePIDController.calculate(getDriveVelocity(),state.speedMetersPerSecond);
		final double driveFeedforward = driveFeedForward
				.calculate(state.speedMetersPerSecond);
		driveOutput += driveFeedforward;

		// Calculate the turning motor output from the turning PID controller.
		final double turnOutput = turningPIDController
				.calculate(getAbsoluteEncoderRad(), state.angle.getRadians());
		//        final double turnFeedforward =
		//         turningFeedForward.calculate(turningPIDController.getSetpoint().velocity);
		setMotors(driveOutput, turnOutput);
	}

	public void setModulePose(Pose2d pose) { m_pose = pose; }

	public Pose2d getModulePose() { return m_pose; }
}
