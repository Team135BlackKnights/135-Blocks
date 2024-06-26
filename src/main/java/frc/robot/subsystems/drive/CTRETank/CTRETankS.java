package frc.robot.subsystems.drive.CTRETank;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.Position;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

public class CTRETankS extends SubsystemChecker implements DrivetrainS {
	private static double kMaxSpeedMetersPerSecond, kWheelDiameter,
			kDriveMotorGearRatio;
	private Pigeon2 pigeon;
	private TalonFX rightLeader, rightFollower, leftLeader, leftFollower;
	private TalonFX[] motors;
	private static DCMotorSim[] motorSimModels;
	private double dtSeconds = 0.02, last_world_linear_accel_x, last_world_linear_accel_y;
	private boolean collisionDetected = false;
	private DifferentialDriveKinematics kinematics;
	private Position<DifferentialDriveWheelPositions> wheelPositions;
	private DifferentialDriveWheelSpeeds wheelSpeeds;
	private Twist2d fieldVelocity = new Twist2d();
	Field2d robotField = new Field2d();
	public Pose2d pose;
	private final VelocityDutyCycle m_motorRequest = new VelocityDutyCycle(0);
	private final VoltageOut m_voltReq = new VoltageOut(0.0);
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, // Use default ramp rate (1 V/s)
					holdVoltage, // Reduce dynamic step voltage to 4 to prevent brownout
					timeout, // Use default timeout (10 s)
					// Log state with Phoenix SignalLogger class
					(state) -> SignalLogger.writeString("state", state.toString())),
			new SysIdRoutine.Mechanism((volts) -> {
				leftLeader.setControl(m_voltReq.withOutput(volts.in(Volts)));
				rightLeader.setControl(m_voltReq.withOutput(volts.in(Volts)));
			}, null, this));
	private DifferentialDrivePoseEstimator poseEstimator;

	/**
	 * Creates a new CTRE tank drivetrain
	 * 
	 * @param container the container to hold the constants
	 * @see CTRETankConstantContainer
	 */
	public CTRETankS(CTRETankConstantContainer container) {
		kMaxSpeedMetersPerSecond = container.getMaxSpeedMetersPerSecond();
		kWheelDiameter = container.getWheelDiameter();
		kDriveMotorGearRatio = container.getDriveGearRatio();
		pigeon = new Pigeon2(container.getPigeonID());
		rightLeader = new TalonFX(container.getRightLeaderID());
		rightFollower = new TalonFX(container.getRightFollowerID());
		leftLeader = new TalonFX(container.getLeftLeaderID());
		leftFollower = new TalonFX(container.getLeftFollowerID());
		motors = new TalonFX[] { rightLeader, rightFollower, leftLeader,
				leftFollower
		};
		motorSimModels = new DCMotorSim[] {
				new DCMotorSim(DCMotor.getKrakenX60Foc(1),
						container.getDriveGearRatio(), 0.001),
				new DCMotorSim(DCMotor.getKrakenX60Foc(1),
						container.getDriveGearRatio(), 0.001),
				new DCMotorSim(DCMotor.getKrakenX60Foc(1),
						container.getDriveGearRatio(), 0.001),
				new DCMotorSim(DCMotor.getKrakenX60Foc(1),
						container.getDriveGearRatio(), 0.001)
		};
		kinematics = new DifferentialDriveKinematics(
				container.getChassisLength());
		pose = new Pose2d(0, 0, getRotation2d());
		poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
				getRotation2d(), getLeftMeters(), getRightMeters(), getPose());
		//do main motors
		TalonFXConfigurator leftConfigurator = leftLeader.getConfigurator();
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
		motorConfig.CurrentLimits.StatorCurrentLimit = 1000;
		motorConfig.MotorOutput.Inverted = InvertedValue
				.valueOf((container.getLeftLeaderReversed()) ? 1 : 0);
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		motorConfig.Feedback.SensorToMechanismRatio = container
				.getDriveEncoderRot2Meter();
		motorConfig.Slot0.kP = .02;
		leftConfigurator.apply(motorConfig);
		//adjust motorConfig for rightSide
		TalonFXConfigurator rightConfig = rightLeader.getConfigurator();
		motorConfig.MotorOutput.Inverted = InvertedValue
				.valueOf((container.getRightLeaderReversed()) ? 1 : 0);
		rightConfig.apply(motorConfig);
		//do followers
		leftFollower.setControl(new Follower(leftLeader.getDeviceID(), false));
		rightFollower.setControl(new Follower(rightLeader.getDeviceID(), false));
		poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
				getRotation2d(), getLeftMeters(), getRightMeters(), pose);
		AutoBuilder.configureLTV(this::getPose, this::resetPose,
				this::getChassisSpeeds, this::setChassisSpeeds, .02,
				new ReplanningConfig(true, true), () -> Robot.isRed, this);
		registerSelfCheckHardware();
	}

	private double getLeftMeters() {
		return (leftLeader.getPosition().getValueAsDouble()
				+ leftFollower.getPosition().getValueAsDouble()) / 2;
	}

	private double getRightMeters() {
		return (rightLeader.getPosition().getValueAsDouble()
				+ rightFollower.getPosition().getValueAsDouble()) / 2;
	}

	private double getLeftVelocity() {
		return (leftLeader.getVelocity().getValueAsDouble()
				+ leftFollower.getVelocity().getValueAsDouble()) / 2;
	}

	private double getRightVelocity() {
		return (rightLeader.getVelocity().getValueAsDouble()
				+ rightFollower.getVelocity().getValueAsDouble()) / 2;
	}

	private DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(getLeftVelocity(),
				getRightVelocity());
	}

	private DifferentialDriveWheelPositions getWheelPositions() {
		return new DifferentialDriveWheelPositions(getLeftMeters(),
				getRightMeters());
	}

	private double metersPerSecondToRotationsPerSecond(
			double velocityMetersPerSecond) {
		double wheelCircumferenceMeters = Math.PI * kWheelDiameter;
		return (velocityMetersPerSecond / wheelCircumferenceMeters)
				* kDriveMotorGearRatio;
	}

	public Rotation2d LastAngle = new Rotation2d();

	@Override
	public void periodic() {
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		wheelSpeeds = getWheelSpeeds();
		poseEstimator.updateWithTime(wheelPositions.getTimestamp(),
				getRotation2d(), wheelPositions.getPositions());
		pose = poseEstimator.getEstimatedPosition();
		DrivetrainS.super.periodic();
		if (Constants.currentMode == Constants.Mode.SIM) {
			for (int i = 0; i < motors.length; i++) {
				var motorSim = motors[i].getSimState();
				motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());
				motorSimModels[i].setInputVoltage(motorSim.getMotorVoltage());
				//There is probably a *2 somewhere, which is causing this .01 instead of .02. Do not remove the TF2 Coconut Solution™.
				motorSimModels[i].update(dtSeconds / 2);
				motorSim.setRawRotorPosition(
						motorSimModels[i].getAngularPositionRotations());
				motorSim.setRotorVelocity(Units.radiansToRotations(
						motorSimModels[i].getAngularVelocityRadPerSec()));
			}
		}
		ChassisSpeeds m_ChassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		fieldVelocity = new Twist2d(linearFieldVelocity.getX(),
				linearFieldVelocity.getY(), m_ChassisSpeeds.omegaRadiansPerSecond);
		boolean collisionDetected = collisionDetected();
		SmartDashboard.putBoolean("Collision Detected", collisionDetected);
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		DifferentialDriveWheelSpeeds wheelSpeeds = kinematics
				.toWheelSpeeds(speeds);
		wheelSpeeds.desaturate(kMaxSpeedMetersPerSecond);
		double leftVelocityRotationsPerSecond = metersPerSecondToRotationsPerSecond(
				wheelSpeeds.leftMetersPerSecond);
		// Set the motor control to the converted velocity
		leftLeader.setControl(
				m_motorRequest.withVelocity(leftVelocityRotationsPerSecond));
		double rightVelocityRotationsPerSecond = metersPerSecondToRotationsPerSecond(
				wheelSpeeds.rightMetersPerSecond);
		// Set the motor control to the converted velocity
		rightLeader.setControl(
				m_motorRequest.withVelocity(rightVelocityRotationsPerSecond));
		if (Constants.currentMode == Constants.Mode.SIM) {
			Pigeon2SimState gyroSim = pigeon.getSimState();
			double angleChange = speeds.omegaRadiansPerSecond * dtSeconds;
			LastAngle = LastAngle.plus(Rotation2d.fromRadians(angleChange));
			gyroSim.setRawYaw(LastAngle.getDegrees());
		}
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(wheelSpeeds);
	}

	@Override
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(getRotation2d(),
				wheelPositions.getPositions(), pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Pose2d getPose() { return pose; }

	@Override
	public void stopModules() {
		leftLeader.setControl(m_motorRequest.withVelocity(0));
		rightLeader.setControl(m_motorRequest.withVelocity(0));
	}

	@Override
	public Rotation2d getRotation2d() {
		return Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
	}

	@Override
	public Command sysIdDynamicTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'sysIdDynamicTurn'");
	}

	@Override
	public Command sysIdQuasistaticTurn(Direction kforwards) {
		throw new UnsupportedOperationException(
				"Unimplemented method 'sysIdQuasistaticTurn'");
	}

	@Override
	public double getCurrent() {
		return motorSimModels[0].getCurrentDrawAmps()
				+ motorSimModels[1].getCurrentDrawAmps()
				+ motorSimModels[2].getCurrentDrawAmps()
				+ motorSimModels[3].getCurrentDrawAmps();
	}

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		return m_sysIdRoutine.quasistatic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		return m_sysIdRoutine.quasistatic(direction);
	}

	@Override
	public void zeroHeading() { pigeon.reset(); }

	@Override
	public boolean isConnected() {
		return pigeon.getFault_Hardware().getValue();
	}
	public boolean collisionDetected() {
		double curr_world_linear_accel_x = pigeon.getAccelerationX().getValueAsDouble();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = pigeon.getAccelerationY().getValueAsDouble();
		double currentJerkY = curr_world_linear_accel_y
				- last_world_linear_accel_y;
		last_world_linear_accel_y = curr_world_linear_accel_y;
		if ((Math.abs(currentJerkX) > DriveConstants.MAX_G)
				|| (Math.abs(currentJerkY) > DriveConstants.MAX_G)) {
			return true;
		}
		return false;
	}
	@Override
	public boolean isCollisionDetected(){
		return collisionDetected;
	}
	@Override
	public double getYawVelocity() {
		if (Constants.currentMode == Constants.Mode.REAL) {
			return Units.degreesToRadians(
					pigeon.getAngularVelocityZWorld().getValueAsDouble());
		}
		return getChassisSpeeds().omegaRadiansPerSecond;
	}

	@Override
	public Twist2d getFieldVelocity() { return fieldVelocity; }

	public void registerSelfCheckHardware() {
		super.registerHardware("IMU", pigeon);
		super.registerHardware("FrontLeft", motors[2]);
		super.registerHardware("FrontRight", motors[0]);
		super.registerHardware("BackLeft", motors[3]);
		super.registerHardware("BackRight", motors[1]);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>(4);
		for (var motor : motors) {
			orchestra.add(motor);
		}
		return orchestra;
	}

	@Override
	public SystemStatus getTrueSystemStatus() { return getSystemStatus(); }

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(1, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					if (getChassisSpeeds().vxMetersPerSecond > 1.2
							|| getChassisSpeeds().vxMetersPerSecond < .8) {
						addFault(
								"[System Check] Forward speed did not reah target speed in time.",
								false, true);
					}
				})).until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public Command getRunnableSystemCheckCommand() {
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() {
		return getOrchestraDevices();
	}

	@Override
	public HashMap<String, Double> getTemps() {
		HashMap<String, Double> tempMap = new HashMap<>();
		tempMap.put("FRTemp", motors[0].getDeviceTemp().getValueAsDouble());
		tempMap.put("BRTemp", motors[1].getDeviceTemp().getValueAsDouble());
		tempMap.put("FLTemp", motors[2].getDeviceTemp().getValueAsDouble());
		tempMap.put("BLTemp", motors[3].getDeviceTemp().getValueAsDouble());
		return tempMap;
	}
}
