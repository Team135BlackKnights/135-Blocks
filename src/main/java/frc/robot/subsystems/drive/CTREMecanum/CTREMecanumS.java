package frc.robot.subsystems.drive.CTREMecanum;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;

public class CTREMecanumS implements DrivetrainS {
	private static Pigeon2 pigeon = new Pigeon2(30);
	private static final TalonFX[] motors = {
			new TalonFX(DriveConstants.kFrontLeftDrivePort),
			new TalonFX(DriveConstants.kFrontRightDrivePort),
			new TalonFX(DriveConstants.kBackLeftDrivePort),
			new TalonFX(DriveConstants.kBackRightDrivePort)
	};
	private static double dtSeconds = 0.02;
	MecanumDriveKinematics kinematics = new MecanumDriveKinematics(
			DriveConstants.kModuleTranslations[0],
			DriveConstants.kModuleTranslations[1],
			DriveConstants.kModuleTranslations[2],
			DriveConstants.kModuleTranslations[3]);
	MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(0, 0, 0,
			0);
	private static final DCMotorSim[] motorSimModels = {
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001),
			new DCMotorSim(DCMotor.getKrakenX60Foc(1),
					DriveConstants.TrainConstants.kDriveMotorGearRatio, 0.001)
	};
	private final VelocityDutyCycle m_motorRequest = new VelocityDutyCycle(0);
	MecanumDriveWheelPositions wheelPositions = new MecanumDriveWheelPositions(0,
			0, 0, 0);
	public Pose2d pose = new Pose2d(0, 0, getRotation2d());
	Field2d robotField = new Field2d();
	MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
			kinematics, getRotation2d(), wheelPositions, pose);
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
				for (TalonFX motor : motors) {
					motor.setControl(m_voltReq.withOutput(volts.in(Volts)));
				}
			}, null, this));

	public CTREMecanumS() {
		for (TalonFX motor : motors) {
			var talonFXConfigurator = motor.getConfigurator();
			TalonFXConfiguration motorConfig = new TalonFXConfiguration();
			motorConfig.CurrentLimits.StatorCurrentLimitEnable = false;
			motorConfig.CurrentLimits.StatorCurrentLimit = 1000;
			motorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
			motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
			motorConfig.Feedback.SensorToMechanismRatio = DriveConstants.TrainConstants.kDriveEncoderRot2Meter;
			motorConfig.Slot0.kP = .02;
			talonFXConfigurator.apply(motorConfig);
		}
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						DriveConstants.kMaxSpeedMetersPerSecond, // Max module speed, in m/s
						DriveConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), () -> Robot.isRed, this // Reference to this subsystem to set requirements
		);

	}

	public double[] getWheelPositionMeters() {
		return new double[] { motors[0].getPosition().getValueAsDouble(),
				motors[1].getPosition().getValueAsDouble(),
				motors[2].getPosition().getValueAsDouble(),
				motors[3].getPosition().getValueAsDouble(),
		};
	}

	public void updateWheelPositions() {
		double[] wheelPos = getWheelPositionMeters();
		wheelPositions.frontLeftMeters = wheelPos[0];
		wheelPositions.frontRightMeters = wheelPos[1];
		wheelPositions.rearLeftMeters = wheelPos[2];
		wheelPositions.rearRightMeters = wheelPos[3];
	}

	private double metersPerSecondToRotationsPerSecond(
			double velocityMetersPerSecond) {
		double wheelCircumferenceMeters = Math.PI
				* DriveConstants.TrainConstants.kWheelDiameter;
		return (velocityMetersPerSecond / wheelCircumferenceMeters)
				* DriveConstants.TrainConstants.kDriveMotorGearRatio;
	}
	public Rotation2d LastAngle = new Rotation2d();

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		// Convert to wheel speeds
		wheelSpeeds = kinematics.toWheelSpeeds(speeds);
		// Get the individual wheel speeds
		double[] velocities = { wheelSpeeds.frontLeftMetersPerSecond,
				wheelSpeeds.frontRightMetersPerSecond,
				wheelSpeeds.rearLeftMetersPerSecond,
				wheelSpeeds.rearRightMetersPerSecond
		};
		if (Constants.currentMode == Constants.Mode.SIM){
			Pigeon2SimState gyroSim = pigeon.getSimState();
			double angleChange = speeds.omegaRadiansPerSecond * dtSeconds;
			LastAngle = LastAngle.plus(Rotation2d.fromRadians(angleChange));
			gyroSim.setRawYaw(LastAngle.getDegrees());
		}
		for (int i = 0; i < motors.length; i++) {
			double velocityRotationsPerSecond = metersPerSecondToRotationsPerSecond(
					velocities[i]);
			// Set the motor control to the converted velocity
			motors[i].setControl(
					m_motorRequest.withVelocity(velocityRotationsPerSecond));
		}
	}

	@Override
	public void periodic() {
		updateWheelPositions();
		poseEstimator.update(getRotation2d(), wheelPositions);
		pose = poseEstimator.getEstimatedPosition();
		robotField.setRobotPose(getPose());
		SmartDashboard.putData(robotField);
		PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/CurrentPose", pose);
			robotField.setRobotPose(pose);
		});
		// Logging callback for target robot pose
		PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
			// Do whatever you want with the pose here
			Logger.recordOutput("Odometry/TrajectorySetpoint", pose);
			robotField.getObject("target pose").setPose(pose);
		});
		// Logging callback for the active path, this is sent as a list of poses
		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			Logger.recordOutput("Odometry/Trajectory",
					poses.toArray(new Pose2d[poses.size()]));
			robotField.getObject("path").setPoses(poses);
		});
		if (Constants.currentMode == Constants.Mode.SIM) {
			for (int i = 0; i < motors.length; i++) {
				var motorSim = motors[i].getSimState();
				motorSim.setSupplyVoltage(12);
				motorSimModels[i].setInputVoltage(motorSim.getMotorVoltage());
				motorSimModels[i].update(dtSeconds);
				motorSim.setRawRotorPosition(
						motorSimModels[i].getAngularPositionRotations());
				motorSim.setRotorVelocity(Units.radiansToRotations(
						motorSimModels[i].getAngularVelocityRadPerSec()));
			}
		}
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return kinematics.toChassisSpeeds(wheelSpeeds);
	}

	@Override
	public void resetPose(Pose2d pose) {
		poseEstimator.resetPosition(getRotation2d(), wheelPositions, pose);
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
		for (int i = 0; i < motors.length; i++) {
			motors[i].setControl(m_motorRequest.withVelocity(0));
		}
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
}