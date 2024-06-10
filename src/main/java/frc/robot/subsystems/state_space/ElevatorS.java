package frc.robot.subsystems.state_space;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

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
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

public class ElevatorS extends SubsystemBase {
	// initialize motors
	private CANSparkBase elevatorMotor;
	private RelativeEncoder elevatorEncoder;
	public static double dtSeconds = .02; // 20 ms
	private static double m_velocity, m_oldPosition, m_position;
	/**
	 * State Space
	 * 
	 * @see FlywheelS FOR IN DEPTH EXPLANATION OF THE STATE-SPACE
	 */
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1));
	Measure<Voltage> holdVoltage = Volts.of(3);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine elevatorIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				setElevatorVolts(volts.in(Volts));
			}, null, this));
	/*
	 * Elevator StateSpace is given a position and velocity, in Meters and Meters
	 * per second
	 * It gets inputted with Volts, and outputs position in Meters.
	 */
	/*
	 * Once the robot is physically made, run tests, and use the
	 * identifyPositionSystem. This will be accurate to the robot rather than CAD.
	 * private final LinearSystem<N2, N1, N1> m_elevatorPlant = LinearSystemId
	 * .identifyPositionSystem(StateSpaceConstants.Elevator.elevatorValueHolder.
	 * getKv(),StateSpaceConstants.Elevator.elevatorValueHolder.getKa());
	 */
	private final LinearSystem<N2, N1, N1> m_elevatorPlant = LinearSystemId
			.createElevatorSystem(DCMotor.getNEO(1),
					StateSpaceConstants.Elevator.carriageMass,
					StateSpaceConstants.Elevator.drumRadius,
					StateSpaceConstants.Elevator.elevatorGearing);
	private final KalmanFilter<N2, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N2(), Nat.N1(), m_elevatorPlant,
			VecBuilder.fill(StateSpaceConstants.Elevator.m_KalmanModelPosition,
					StateSpaceConstants.Elevator.m_KalmanModelVelocity), // How accurate we
			// think our model is, in meters and meters/second.
			VecBuilder.fill(StateSpaceConstants.Elevator.m_KalmanEncoder), // How accurate we think our encoder position
			// data is. In this case we very highly trust our encoder position reading.
			dtSeconds);
	private final LinearQuadraticRegulator<N2, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			m_elevatorPlant,
			VecBuilder.fill(Units.inchesToMeters(1.0), Units.inchesToMeters(10.0)), // qelms. Position
			// and velocity error tolerances, in meters and meters per second. We care about
			// pos more than velocity.
			VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
			// heavily penalize control effort
			dtSeconds);
	private final LinearSystemLoop<N2, N1, N1> m_loop = new LinearSystemLoop<>(
			m_elevatorPlant, m_controller, m_observer, 12.0, dtSeconds);
	/**
	 * For Trapezoid Profile/Sim/Mech2d
	 * 
	 * @see ArmS
	 */
	private final TrapezoidProfile m_profile = new TrapezoidProfile(
			new TrapezoidProfile.Constraints(StateSpaceConstants.Elevator.maxSpeed,
					StateSpaceConstants.Elevator.maxAcceleration)); // Max elevator speed and acceleration.
	private TrapezoidProfile.State m_lastProfiledReference = new TrapezoidProfile.State();
	private static TrapezoidProfile.State goal = new TrapezoidProfile.State(
			StateSpaceConstants.Elevator.startingPosition, 0);
	private ElevatorSim simElevator = new ElevatorSim(m_elevatorPlant,
			DCMotor.getNEO(1), StateSpaceConstants.Elevator.startingPosition,
			StateSpaceConstants.Elevator.maxPosition, false,
			StateSpaceConstants.Elevator.startingPosition);
	// Create a Mechanism2d visualization of the elevator
	private final Mechanism2d m_mech2d = new Mechanism2d(
			DriveConstants.kChassisWidth, DriveConstants.kChassisLength);
	private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot(
			"Elevator Root", StateSpaceConstants.Elevator.physicalX,
			StateSpaceConstants.Elevator.physicalY);
	private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot
			.append(new MechanismLigament2d("Elevator",
					simElevator.getPositionMeters(), 90));

	public ElevatorS() {
		switch (StateSpaceConstants.Elevator.ElevatorMotorVendor) {
			case NEO_SPARK_MAX:
				elevatorMotor = new CANSparkMax(StateSpaceConstants.Arm.kMotorID, MotorType.kBrushless);
				break;
			case VORTEX_SPARK_FLEX:
				elevatorMotor = new CANSparkFlex(StateSpaceConstants.Arm.kMotorID, MotorType.kBrushless);
				break;
			default:
				System.out.println("Unidentified motor type, real-world applications probably will not work");
				break;
		}
		// initalize elevator motor
		elevatorMotor.setInverted(StateSpaceConstants.Elevator.inverted);
		elevatorMotor.setIdleMode(StateSpaceConstants.Elevator.mode);
		elevatorEncoder = elevatorMotor.getEncoder();
		elevatorEncoder.setPositionConversionFactor(1 / StateSpaceConstants.Elevator.elevatorGearing);
		elevatorEncoder.setVelocityConversionFactor(1 / StateSpaceConstants.Elevator.elevatorGearing);
		elevatorMotor.burnFlash();
		// reset our position
		m_loop.reset(VecBuilder.fill(getDistance(), getVelocity()));
		m_lastProfiledReference = new TrapezoidProfile.State(getDistance(),
				getVelocity());
	}

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return elevatorIdRoutine.quasistatic(direction)
				.onlyWhile(withinLimits(direction));
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return elevatorIdRoutine.dynamic(direction)
				.onlyWhile(withinLimits(direction));
	}

	public static double getDistance() {
		return m_position;
	}

	public static double getSetpoint() {
		return goal.position;
	}

	public double getVelocity() {
		return m_velocity;
	}

	/**
	 * Given a direction, is the elevator currently within LIMITS
	 * 
	 * @param direction travelling
	 * @return true or false SUPPLIER
	 */
	public BooleanSupplier withinLimits(SysIdRoutine.Direction direction) {
		BooleanSupplier returnVal;
		if (direction.toString() == "kReverse") {
			if (getDistance() < StateSpaceConstants.Elevator.startingPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		// second set of conditionals (below) checks to see if the elevator is within
		// the hard limits, and stops it if it is
		if (direction.toString() == "kForward") {
			if (getDistance() > StateSpaceConstants.Elevator.maxPosition) {
				returnVal = () -> false;
				return returnVal;
			}
		}
		// otherwise, we're in bounds!
		returnVal = () -> true;
		return returnVal;
	}

	/*
	 * Create a state that is the base position
	 */
	public TrapezoidProfile.State startingState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Elevator.startingPosition, 0);
	}

	/*
	 * Create a state that is the MAXIMUM position
	 */
	public TrapezoidProfile.State maxState() {
		return new TrapezoidProfile.State(
				StateSpaceConstants.Elevator.maxPosition, 0);
	}

	/**
	 * For cleanliness of code, create State is in ElevatorS, called everywhere
	 * else.
	 * 
	 * @param position in METERS
	 * @return state, pass through to moveElevator.
	 */
	public TrapezoidProfile.State createState(double position) {
		return new TrapezoidProfile.State(position, 0);
	}

	// Also overload the function to accept both angle in METERS and m/s
	/**
	 * @param position in METERS
	 * @param velocity in METERS/SECOND
	 * @return state, pass through to deployIntake.
	 */
	public TrapezoidProfile.State createState(double position, double velocity) {
		return new TrapezoidProfile.State(position, velocity);
	}

	/**
	 * Checks if close to state, with less than specified meters
	 * 
	 * @param meters error in meters
	 * @return true if at state
	 */
	public boolean isAtState(double meters) {
		if (Math.abs(getError()) < meters) {
			return true;
		}
		return false;
	}

	/**
	 * @return error in meters
	 */
	public double getError() {
		return m_position - goal.position;
	}

	public void moveElevator(TrapezoidProfile.State state) {
		goal = state;
	}

	public void updateEncoders() {
		switch (Constants.currentMode) {
			case REAL:
				m_position = elevatorEncoder.getPosition();
				m_velocity = m_position - m_oldPosition; // since called every 20 ms
				m_oldPosition = m_position;
				break;
			default:
				m_position = simElevator.getPositionMeters();
				m_velocity = simElevator.getVelocityMetersPerSecond();
				m_elevatorMech2d.setLength(m_position);
				break;
		}
	}

	public void setElevatorVolts(double volts) {
		elevatorMotor.setVoltage(volts);
	}

	@Override
	public void periodic() {
		updateEncoders();
		if (StateSpaceConstants.debug) {
			SmartDashboard.putNumber("Position Error Ele.", getError());
			SmartDashboard.putNumber("SETPOINT Ele.", goal.position);
			SmartDashboard.putNumber("CURRENT WANTED Ele.",
					m_lastProfiledReference.position);
		}
		m_lastProfiledReference = m_profile.calculate(dtSeconds,
				m_lastProfiledReference, goal); // calculate where it SHOULD be.
		m_loop.setNextR(m_lastProfiledReference.position,
				m_lastProfiledReference.velocity); // Tell our motors to get there
		// Correct our Kalman filter's state vector estimate with encoder data ONLY if
		// real
		if (Constants.currentMode == Mode.REAL) {
			m_loop.correct(VecBuilder.fill(getDistance()));
		}
		// Update our LQR to generate new voltage commands and use the voltages to
		// predict the next
		// state with out Kalman filter.
		m_loop.predict(dtSeconds);
		// Send the new calculated voltage to the motors.
		double nextVoltage = m_loop.getU(0);
		switch (Constants.currentMode) {
			case REAL:
				elevatorMotor.setVoltage(nextVoltage);
				break;
			default:
				simElevator.setInputVoltage(nextVoltage);
				simElevator.update(dtSeconds);
				break;
		}
		// Push the mechanism to AdvantageScope
		Logger.recordOutput("ElevatorMechanism", m_mech2d);
		// calcualate arm pose
		var elevatorPose = new Pose3d(StateSpaceConstants.Elevator.simX, StateSpaceConstants.Elevator.simY,
				StateSpaceConstants.Elevator.simZ,
				new Rotation3d(0, 0, 0.0));
		Logger.recordOutput("Mechanism3d/Elevator/", elevatorPose);
	}
}
