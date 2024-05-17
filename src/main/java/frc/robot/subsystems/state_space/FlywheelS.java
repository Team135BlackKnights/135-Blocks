package frc.robot.subsystems.state_space;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelS extends SubsystemBase {
	//initialize motors
	private CANSparkMax flywheel = new CANSparkMax(
			StateSpaceConstants.Flywheel.kMotorID, MotorType.kBrushless);
	//encoders
	private static RelativeEncoder flywheelEncoder;
	//System ID Routine
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); // for going FROM ZERO PER SECOND, this is 1v per 1sec.
	Measure<Voltage> holdVoltage = Volts.of(7); //what voltage should I hold during Quas test?
	Measure<Time> timeout = Seconds.of(10); //how many total seconds should I run the test, unless interrupted?
	//update cycle time
	private static double dtSeconds = .02; //20 ms
	/**
	 * We cannot have multiple sysIdRoutines run on the same robot-cycle. If we
	 * want to run this and another sysIdRoutine, you must first POWER CYCLE the
	 * robot.
	 */
	SysIdRoutine sysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				flywheel.setVoltage(volts.in(Volts)); //no touch!
			}, null // No log consumer, since data is recorded by URCL
					, this));
	/**
	 * This Plant holds a state-space model of our flywheel. It has the following
	 * properties: States: Velocity, in Rad/s. (will match Output) Inputs: Volts.
	 * Outputs: Velcoity, in Rad/s Flywheels require either an MOI or Kv and Ka,
	 * which are found using SysId.
	 */
	private final static LinearSystem<N1, N1, N1> flywheelPlant = LinearSystemId
			.identifyVelocitySystem(
					StateSpaceConstants.Flywheel.kFlywheelVVoltSecondsPerRotation,
					StateSpaceConstants.Flywheel.kFlywheelAVoltSecondsSquaredPerRotation);
	/**
	 * Kalman filters are optional, and help to predict the actual state of the
	 * system given a noisy encoder (which all encoders are!) Takes two empty
	 * N1's, our plant, and then our model accuracy in St. Devs and our Encoder
	 * value in St. Devs. Higher value means less trust, as an example 1 means we
	 * trust it 68% of all time.
	 */
	private final static KalmanFilter<N1, N1, N1> m_observer = new KalmanFilter<>(
			Nat.N1(), Nat.N1(), flywheelPlant,
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_KalmanModel),
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_KalmanEncoder),
			dtSeconds);
	/**
	 * A Linear Quadratic Regulator (LQR) controls linear systems. It minimizes
	 * cost, or voltage, to minimize state error. So, get to a position as fast,
	 * and efficient, as possible Our Qelms, or Q, is our PENALTY for deviation.
	 * Qelms is for our output, here being RPM, and R (Volts) is our penalty for
	 * control effort. Basically, high either of these means that it will take
	 * that more into account. A Qelms value of 100 means it REALLY cares about
	 * our RPM, and penalizes HEAVILY for incorrectness.
	 */
	private final static LinearQuadraticRegulator<N1, N1, N1> m_controller = new LinearQuadraticRegulator<>(
			flywheelPlant,
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_LQRQelms),
			VecBuilder.fill(StateSpaceConstants.Flywheel.m_LQRRVolts), dtSeconds);
	/**
	 * A state-space loop combines a controller, observer, feedforward, and plant
	 * all into one. It does everything! The random 12 is the upper limit on
	 * Voltage the loop can output. Never require more than 12 V, because.. we
	 * can't give it!
	 */
	private final static LinearSystemLoop<N1, N1, N1> m_loop = new LinearSystemLoop<>(
			flywheelPlant, m_controller, m_observer, 12, dtSeconds);
	/**
	 * Now, we define how it reacts in simulation. A gearing greater than 1 is a
	 * reduction.
	 */
	private final FlywheelSim flywheelSim = new FlywheelSim(flywheelPlant,
			DCMotor.getNEO(1), 1 / StateSpaceConstants.Flywheel.flywheelGearing);
	private double nextVoltage;

	public FlywheelS() {
		//initalize our motor
		flywheel.setInverted(StateSpaceConstants.Flywheel.inverted);
		flywheel.setIdleMode(StateSpaceConstants.Flywheel.mode);
		flywheelEncoder = flywheel.getEncoder();
		flywheelEncoder.setVelocityConversionFactor(
				StateSpaceConstants.Flywheel.flywheelGearing);
		flywheelEncoder.setPositionConversionFactor(
				StateSpaceConstants.Flywheel.flywheelGearing);
		flywheel.burnFlash();
		//Tell our controllers that the encoders have a 19.5 ms delay (sparks)
		m_controller.latencyCompensate(flywheelPlant, dtSeconds, .0195);
		m_loop.setNextR(0); //go to zero
		m_loop.reset(getEncoderRad());
		//that^ may not be in radians.
	}

	private Vector<N1> getEncoderRad() {
		return VecBuilder
				.fill(Units.degreesToRadians(flywheelEncoder.getVelocity()));
	}

	public static double flywheelVelocity;

	@Override
	public void periodic() {
		m_loop.correct(getEncoderRad());
		m_loop.predict(dtSeconds);
		nextVoltage = m_loop.getU(0); //get model's control output
		switch (Constants.currentMode) {
		case REAL:
			flywheel.setVoltage(nextVoltage);
			flywheelVelocity = flywheelEncoder.getVelocity();
			break;
		default:
			flywheelSim.setInput(nextVoltage);
			flywheelSim.update(dtSeconds);
			flywheelVelocity = flywheelSim.getAngularVelocityRPM();
			break;
		}
		if (StateSpaceConstants.debug) {
			SmartDashboard.putNumber("Flywheel Speed", flywheelVelocity);
		}
	}

	/**
	 * @param direction forward/reverse ("kForward" or "kReverse")
	 * @return command which runs wanted test
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.quasistatic(direction);
	}

	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return sysIdRoutine.dynamic(direction);
	}

	//TODO: test me!
	public static double getSpeedError(double setRPM) {
		return Math.abs(flywheelVelocity - setRPM);
		// return m_bottomLoop.getError(0); //very low chance this is the wrong call
	}
	/*
	 * Set RPM desired.
	 * @param speed in RPM
	 */
	public void setRPM(double rpm){
		m_loop.setNextR(VecBuilder.fill(rpm));
	}
	//Sim Only
	public double getDrawnCurrentAmps() {
		return flywheelSim.getCurrentDrawAmps();
	}
}