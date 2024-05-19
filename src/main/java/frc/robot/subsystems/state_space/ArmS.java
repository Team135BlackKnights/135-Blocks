package frc.robot.subsystems.state_space;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.state_space.StateSpaceConstants;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import java.util.function.BooleanSupplier;

public class ArmS extends SubsystemBase {
	//initalize motors
	private CANSparkMax armMotor = new CANSparkMax(
			StateSpaceConstants.Arm.kMotorID, MotorType.kBrushless);
	private RelativeEncoder armEncoder;
	//update cycle time
	public static double dtSeconds = .02; //20 ms
	//live tuning, only enabled in Debug.
	/**
	 * State Space
	 * 
	 * @see FlywheelS FOR IN DEPTH EXPLANATION OF THE STATE-SPACE VARIABLE
	 */
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
	Measure<Voltage> holdVoltage = Volts.of(3);
	Measure<Time> timeout = Seconds.of(10);
	/**
	 * We cannot have multiple sysIdRoutines run on the same robot-cycle. If we
	 * want to run this and another sysIdRoutine, you must first POWER CYCLE the
	 * robot.
	 */
	SysIdRoutine armIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				setIntakeMotorVolts(volts.in(Volts));
			}, null, this));
	//using sysId
	/*
	 * All Arm Statespace uses an N2 at the first position, because we care about velocity AND position of the arm.
	 * First position in the Nat is for Position, second is Velocity.
	 */
	//private final LinearSystem<N2,N1,N1> m_armPlant = 
	//   LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), SingleJointedArmSim.estimateMOI(Units.inchesToMeters(10), Units.lbsToKilograms(6)), 150);
	private final LinearSystem<N2, N1, N1> m_armPlant = LinearSystemId
			.identifyPositionSystem(StateSpaceConstants.Arm.armValueHolder.getKv(),
					StateSpaceConstants.Arm.armValueHolder.getKa());
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_armPlant,
			VecBuilder.fill(StateSpaceConstants.Arm.m_KalmanModelPosition,
					StateSpaceConstants.Arm.m_KalmanModelVelocity),
			VecBuilder.fill(StateSpaceConstants.Arm.m_KalmanEncoder), dtSeconds);
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_armPlant,
			VecBuilder.fill(StateSpaceConstants.Arm.m_LQRQelmsPosition,
					StateSpaceConstants.Arm.m_LQRQelmsVelocity),
			VecBuilder.fill(StateSpaceConstants.Arm.m_LQRRVolts), dtSeconds);
	// lower if using notifiers.
	// The state-space loop combines a controller, observer, feedforward and plant for easy control.
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_armPlant, m_controller, m_observer, 12.0, dtSeconds);
	private static double m_velocity, m_oldPosition, m_position;
	/**
	 * Create a TrapezoidProfile, which holds constraints and states of our arm,
	 * allowing smooth motion control for the arm. Created with constraints based
	 * on the motor's free speed, but this will vary for every system, try tuning
	 * these.
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(StateSpaceConstants.Arm.maxSpeed, //placeholder
					StateSpaceConstants.Arm.maxAcceleration));
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	//set our starting position for the arm
	/*
	 * TrapezoidProfile States are basically just a position in rads with a velocity in Rad/s
	 * Here, we provide our starting position.
	 */
	private static TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.Arm.startingPosition, 0);
	/**
	 * Constructs a single-jointed arm simulation with the given parameters.
	 * 
	 * @param armGearing       The gearing ratio of the arm. >1 means reduction.
	 * @param armLength        The length of the arm (in meters)
	 * @param minPosition      The minimum allowed position of the arm (in
	 *                            radians).
	 * @param maxPosition      The maximum allowed position of the arm (in
	 *                            radians).
	 * @param startingPosition The initial position of the arm (in radians).
	 *                            Calculates the position, velocity, and
	 *                            acceleration of the arm based on the applied
	 *                            motor voltage and time step. Provides current
	 *                            position, velocity, and voltage outputs for
	 *                            monitoring and control purposes.
	 */
	private SingleJointedArmSim simArm = new SingleJointedArmSim(m_armPlant,
			DCMotor.getNEO(1), StateSpaceConstants.Arm.armGearing,
			StateSpaceConstants.Arm.armLength,
			StateSpaceConstants.Arm.startingPosition,
			StateSpaceConstants.Arm.maxPosition, false,
			StateSpaceConstants.Arm.startingPosition);
	// Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
	/*
	 * Mechanism2d is really just an output of the robot, used to debug arm movement in simulation.
	 * the Mech2d itself is the "canvas" the mechanisms (like arms) are put on. It will always be your chassis.
	 * A Root2d is the point at which the mechanism rotates, or starts at. Elevators are different, but here we
	 * simply get it's X and Y according to the robot, then make that the root.
	 * Finally, we make the Ligament itself, and append this to the root point, basically putting an object at
	 * an origin point RELATIVE to the robot
	 * It takes the current position of the arm, and is the only thing updated constantly because of that
	 */
	private final Mechanism2d m_mech2d = new Mechanism2d(
			DriveConstants.kChassisWidth, DriveConstants.kChassisLength);
	private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot",
			StateSpaceConstants.Arm.physicalX, StateSpaceConstants.Arm.physicalY);
	private final MechanismLigament2d m_arm = m_armPivot.append(
			new MechanismLigament2d("Arm", StateSpaceConstants.Arm.armLength,
					Units.radiansToDegrees(m_lastProfiledReference.position), 1, new Color8Bit(Color.kYellow)));

	public ArmS() {
		//initalize arm motor
		armMotor.setInverted(StateSpaceConstants.Arm.inverted);
		armMotor.setIdleMode(StateSpaceConstants.Arm.mode);
		armEncoder = armMotor.getEncoder();
		armEncoder.setPositionConversionFactor(
				1 / StateSpaceConstants.Arm.armGearing);
		armEncoder.setVelocityConversionFactor(
				1 / StateSpaceConstants.Arm.armGearing);
		armMotor.burnFlash();
		//reset our position (getDistance just because it's zero usually, and IF the robot died on field and rebooted, it'd still be accurate.)
		m_loop.reset(VecBuilder.fill(getDistance(), getVelocity()));
		m_lastProfiledReference = new TrapezoidProfile.State(getDistance(),
				getVelocity());
	}

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return armIdRoutine.quasistatic(direction)
				.onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return armIdRoutine.dynamic(direction).onlyWhile(withinLimits(direction));
	}

	public static double getDistance() { return m_position; }

	public static double getSetpoint() { return goal.position; }

	public double getVelocity() { return m_velocity; }

	/**
	 * Given a direction, is the arm currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < StateSpaceConstants.Arm.startingPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//second set of conditionals (below) checks to see if the arm is within the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > StateSpaceConstants.Arm.maxPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		//otherwise, we're in bounds!
		returnVal = () -> true;
		return returnVal;
	}

	/*
	 * Create a state that is the base position
	 */
	public TrapezoidProfile.State startingState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Arm.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(StateSpaceConstants.Arm.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in ArmS, called everywhere else.
	 * 
	 * @param angle IN RADS
	 * @return state, pass through to deployIntake.
	 */
	public TrapezoidProfile.State createState(double angle) {
		return new TrapezoidProfile.State(angle, 0);
	}

	//Also overload the function to accept both angle in RADS and rad/s
	/**
	 * @param position in RADIANS
	 * @param velocity in RADIANS/SECOND
	 * @return state, pass through to deployIntake.
	 */
	public TrapezoidProfile.State createState(double position, double velocity) {
		return new TrapezoidProfile.State(position, velocity);
	}

	/**
	 * Checks if close to state, with less than specified degrees
	 * 
	 * @param degrees error in degrees
	 * @return true if at state
	 */
	public boolean isAtState(double degrees) {
		if (Math.abs(getError()) < Units.degreesToRadians(degrees)) {
			return true;
		}
		return false;
	}

	/**
	 * @return error in radians
	 */
	public double getError() { return m_position - goal.position; }

	public void deployIntake(TrapezoidProfile.State state) { goal = state; }

	public void updateEncoders() {
		switch (Constants.currentMode) {
		case REAL:
			m_position = armEncoder.getPosition();
			m_velocity = m_position - m_oldPosition; //since called every 20 ms
			m_oldPosition = m_position;
			break;
		default:
			m_position = simArm.getAngleRads();
			m_velocity = simArm.getVelocityRadPerSec();
			m_arm.setAngle(m_position);
			break;
		}
	}

	public void setIntakeMotorVolts(double volts) { armMotor.setVoltage(volts); }

	@Override
	public void periodic() {
		updateEncoders();
		m_lastProfiledReference = m_profile.calculate(dtSeconds,
				m_lastProfiledReference, goal); //calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); //Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data ONLY if real
		if (Constants.currentMode == Mode.REAL) {
			m_loop.correct(VecBuilder.fill(getDistance()));
		}
		// Update our LQR to generate new voltage commands and use the voltages to predict the next
		// state with out Kalman filter.
		m_loop.predict(dtSeconds);
		// Send the new calculated voltage to the motors.
		double nextVoltage = m_loop.getU(0);
		switch (Constants.currentMode) {
		case REAL:
			armMotor.setVoltage(nextVoltage);
			break;
		default:
			simArm.setInputVoltage(nextVoltage);
			simArm.update(dtSeconds);
			break;
		}
		//Push the mechanism to AdvantageScope
		Logger.recordOutput("ArmMechanism", m_mech2d);
		//calcualate arm pose
		var armPose = new Pose3d(StateSpaceConstants.Arm.simX, StateSpaceConstants.Arm.simY, StateSpaceConstants.Arm.simZ,
				new Rotation3d(0, -m_lastProfiledReference.position, 0.0));
		Logger.recordOutput("Mechanism3d/Arm/", armPose);
		if (StateSpaceConstants.debug) {
			SmartDashboard.putNumber("Angle Error ARM", getError());
			SmartDashboard.putNumber("SETPOINT ARM",
					Units.radiansToDegrees(goal.position));
			SmartDashboard.putNumber("CURRENT WANTED ARM",
					Units.radiansToDegrees(m_lastProfiledReference.position));
			SmartDashboard.putNumber("pos ARM",
					Units.radiansToDegrees(m_position));
		}
	}
}
