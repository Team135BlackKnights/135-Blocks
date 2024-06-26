package frc.robot.subsystems.drive.REVMecanum;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.Collections;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import org.ejml.simple.UnsupportedOperation;
import org.littletonrobotics.junction.Logger;
import java.util.HashMap;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort.Port;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.Position;

public class REVMecanumS extends SubsystemChecker implements DrivetrainS {
	private static CANSparkBase[] sparkMotors = new CANSparkBase[4];
	private static int[] motorIDs;
	Field2d robotField = new Field2d();
	private double m_simYaw, last_world_linear_accel_x, last_world_linear_accel_y;;
	private static DCMotorSim[] motorSims = new DCMotorSim[4];
	private static RelativeEncoder[] wheelRelativeEncoders = new RelativeEncoder[4];
	private static AHRS gyro;
	private static MecanumDriveKinematics driveKinematics;
	private static MecanumDrivePoseEstimator drivePoseEstimator;
	private static Position<MecanumDriveWheelPositions> wheelPositions;
	private static double maxDriveVelMetersPerSec, kDriveBaseRadius;
	private static Translation2d[] kModuleTranslations;
	private static double gearing, kWheelDiameterMeters;
	private boolean collisionDetected = false;
	private static Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
	Measure<Velocity<Voltage>> rampRate = Volts.of(1).per(Seconds.of(1)); //for going FROM ZERO PER SECOND
	Measure<Voltage> holdVoltage = Volts.of(4);
	Measure<Time> timeout = Seconds.of(10);
	SysIdRoutine sysIdRoutineDrive = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				for (int i = 0; i < 4; i++) {
					sparkMotors[i].setVoltage(volts.in(Volts));
				}
			}, null // No log consumer, since data is recorded by URCL
					, this));
	SysIdRoutine sysIdRoutineTurn = new SysIdRoutine(
			new SysIdRoutine.Config(rampRate, holdVoltage, timeout,
					(state) -> Logger.recordOutput("SysIdTestState",
							state.toString())),
			new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
				sparkMotors[0].setVoltage(-volts.in(Volts));
				sparkMotors[1].setVoltage(-volts.in(Volts));
				sparkMotors[2].setVoltage(volts.in(Volts));
				sparkMotors[3].setVoltage(volts.in(Volts));
			}, null // No log consumer, since data is recorded by URCL
					, this));

	/**
	 * Constructs a REV Mecanum Drivetrain.
	 * 
	 * @param container the container that holds the constants
	 * @see REVMecanumConstantContainer
	 */
	public REVMecanumS(REVMecanumConstantContainer container) {
		motorIDs = new int[] { container.getFrontLeftID(),
				container.getFrontRightID(), container.getBackLeftID(),
				container.getBackRightID()
		};
		gearing = container.getGearing();
		kWheelDiameterMeters = container.getWheelDiameters();
		kModuleTranslations = container.getTranslation2ds();
		kDriveBaseRadius = container.getDriveBaseRadius();
		for (int i = 0; i < 4; i++) {
			switch (DriveConstants.robotMotorController) {
			case NEO_SPARK_MAX:
				sparkMotors[i] = new CANSparkMax(motorIDs[i], MotorType.kBrushless);
				motorSims[i] = new DCMotorSim(DCMotor.getNEO(1), gearing, .001);
				maxDriveVelMetersPerSec = (5676 / 60.0) / gearing
						* (Math.PI * kWheelDiameterMeters);
				break;
			case VORTEX_SPARK_FLEX:
				sparkMotors[i] = new CANSparkFlex(motorIDs[i],
						MotorType.kBrushless);
				motorSims[i] = new DCMotorSim(DCMotor.getNeoVortex(1), gearing,
						.001);
				maxDriveVelMetersPerSec = (6784 / 60.0) / gearing
						* (Math.PI * kWheelDiameterMeters);
				break;
			default:
				throw new UnsupportedOperation("no REV motortype found");
			}
			sparkMotors[i].setIdleMode(IdleMode.kBrake);
			sparkMotors[i].enableVoltageCompensation(12);
			sparkMotors[i].setSmartCurrentLimit(i, i);
			sparkMotors[i].clearFaults();
			sparkMotors[i].burnFlash();
			wheelRelativeEncoders[i] = sparkMotors[i].getEncoder();
			wheelRelativeEncoders[i].setPositionConversionFactor(
					gearing * kWheelDiameterMeters * Math.PI);
			wheelRelativeEncoders[i].setVelocityConversionFactor(
					gearing * kWheelDiameterMeters * Math.PI);
		}
		gyro = new AHRS(Port.kUSB);
		driveKinematics = new MecanumDriveKinematics(kModuleTranslations[0],
				kModuleTranslations[1], kModuleTranslations[2],
				kModuleTranslations[3]);
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		drivePoseEstimator = new MecanumDrivePoseEstimator(driveKinematics,
				getRotation2d(), wheelPositions.getPositions(), pose);
		AutoBuilder.configureHolonomic(this::getPose, // Robot pose supplier
				this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
				this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
				this::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
				new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
						new PIDConstants(10, 0.0, 0.0), // Translation PID constants // We didn't have the chance to optimize PID constants so there will be some error in autonomous until these values are fixed
						new PIDConstants(5, 0.0, 0.0), // Rotation PID constants
						maxDriveVelMetersPerSec, // Max module speed, in m/s
						kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
						new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
				), () -> Robot.isRed, this // Reference to this subsystem to set requirements
		);
		registerSelfCheckHardware();
	}

	public MecanumDriveWheelSpeeds getWheelSpeeds() {
		return new MecanumDriveWheelSpeeds(wheelRelativeEncoders[0].getVelocity(),
				wheelRelativeEncoders[1].getVelocity(),
				wheelRelativeEncoders[2].getVelocity(),
				wheelRelativeEncoders[3].getVelocity());
	}

	public MecanumDriveWheelPositions getWheelPositions() {
		return new MecanumDriveWheelPositions(
				wheelRelativeEncoders[0].getPosition(),
				wheelRelativeEncoders[1].getPosition(),
				wheelRelativeEncoders[2].getPosition(),
				wheelRelativeEncoders[3].getPosition());
	}

	@Override
	public void periodic() {
		wheelPositions = getPositionsWithTimestamp(getWheelPositions());
		drivePoseEstimator.updateWithTime(wheelPositions.getTimestamp(),
				getRotation2d(), wheelPositions.getPositions());
		if (Constants.currentMode == Constants.Mode.SIM) {
			for (int i = 0; i < 4; i++) {
				motorSims[i].setInputVoltage(sparkMotors[i].get() * RobotController.getBatteryVoltage());
				//There is probably a *2 somewhere, which is causing this .01 instead of .02. Do not remove the TF2 Coconut Solution™.
				motorSims[i].update(.01);
				wheelRelativeEncoders[i]
						.setPosition(motorSims[i].getAngularPositionRotations());
			}
		}
		DrivetrainS.super.periodic();
		boolean collisionDetected = collisionDetected();
		SmartDashboard.putBoolean("Collision Detected", collisionDetected);	}

	@Override
	public double getCurrent() {
		return motorSims[0].getCurrentDrawAmps()
				+ motorSims[1].getCurrentDrawAmps()
				+ motorSims[2].getCurrentDrawAmps()
				+ motorSims[3].getCurrentDrawAmps();
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		MecanumDriveWheelSpeeds indSpeeds = driveKinematics.toWheelSpeeds(speeds);
		indSpeeds.desaturate(maxDriveVelMetersPerSec);
		sparkMotors[0]
				.set(indSpeeds.frontLeftMetersPerSecond / maxDriveVelMetersPerSec);
		sparkMotors[1]
				.set(indSpeeds.frontRightMetersPerSecond / maxDriveVelMetersPerSec);
		sparkMotors[2]
				.set(indSpeeds.rearLeftMetersPerSecond / maxDriveVelMetersPerSec);
		sparkMotors[3]
				.set(indSpeeds.rearRightMetersPerSecond / maxDriveVelMetersPerSec);
		if (Constants.currentMode == Constants.Mode.SIM) {
			m_simYaw += speeds.omegaRadiansPerSecond * 1;
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(
					SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			// NavX expects clockwise positive, but sim outputs clockwise negative
			angle.set(Math.IEEEremainder(-Units.radiansToDegrees(m_simYaw), 360));
		}
	}

	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return driveKinematics.toChassisSpeeds(getWheelSpeeds());
	}

	@Override
	public void resetPose(Pose2d pose) {
		drivePoseEstimator.resetPosition(getRotation2d(), getWheelPositions(),
				pose);
	}

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		drivePoseEstimator.addVisionMeasurement(pose, timestamp);
	}

	@Override
	public Pose2d getPose() { return drivePoseEstimator.getEstimatedPosition(); }

	@Override
	public void stopModules() {
		for (int i = 0; i < 4; i++) {
			sparkMotors[i].set(0);
		}
	}

	@Override
	public Rotation2d getRotation2d() {
		if (Constants.currentMode == Constants.Mode.SIM) {
			return Rotation2d.fromDegrees(m_simYaw);
		}
		return Rotation2d.fromDegrees(gyro.getAngle());
	}

	/**
	 * @exception THIS CANNOT BE DONE USING MECANUM!
	 */
	@Override
	public Command sysIdDynamicTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Impossible method 'sysIdDynamicTurn' for DriveTrain mecanum");
	}

	/**
	 * @exception THIS CANNOT BE DONE USING MECANUM!
	 */
	@Override
	public Command sysIdQuasistaticTurn(Direction kreverse) {
		throw new UnsupportedOperationException(
				"Impossible method 'sysIdQuasistaticTurn' for DriveTrain mecanum");
	}

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		return sysIdRoutineDrive.dynamic(direction);
	}

	@Override
	public void zeroHeading() { gyro.reset(); }

	@Override
	public boolean isConnected() { return gyro.isConnected(); }

	@Override
	public double getYawVelocity() {
		return getChassisSpeeds().omegaRadiansPerSecond; //??
	}

	@Override
	public Twist2d getFieldVelocity() {
		ChassisSpeeds m_ChassisSpeeds = getChassisSpeeds();
		Translation2d linearFieldVelocity = new Translation2d(
				m_ChassisSpeeds.vxMetersPerSecond,
				m_ChassisSpeeds.vyMetersPerSecond).rotateBy(getRotation2d());
		return new Twist2d(linearFieldVelocity.getX(), linearFieldVelocity.getY(),
				m_ChassisSpeeds.omegaRadiansPerSecond);
	}
		public void registerSelfCheckHardware() {
    super.registerHardware("IMU", gyro);
	 super.registerHardware("FrontLeft", sparkMotors[0]);
	 super.registerHardware("FrontRight", sparkMotors[1]);
	 super.registerHardware("BackLeft", sparkMotors[2]);
	 super.registerHardware("BackRight", sparkMotors[3]);
	 
   }
	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	 }

	@Override
	public SystemStatus getTrueSystemStatus(){
		return getSystemStatus();
	}
	@Override
	public Command getRunnableSystemCheckCommand(){
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() { 
		return getOrchestraDevices();
	}
	@Override
	protected Command systemCheckCommand() { 
	return Commands.sequence(run(() -> setChassisSpeeds(new ChassisSpeeds(0,0,0.5))).withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0,0,-0.5))).withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0,1,0))).withTimeout(1.0),
				runOnce(() -> {
					if (getChassisSpeeds().vyMetersPerSecond > 1.2 || getChassisSpeeds().vyMetersPerSecond < .8){
						addFault("[System Check] Strafe speed did not reah target speed in time.", false,true);
					}
				}),
				run(() -> setChassisSpeeds(new ChassisSpeeds(1,0,0))).withTimeout(1.0),
				runOnce(() -> {
					if (getChassisSpeeds().vxMetersPerSecond > 1.2 || getChassisSpeeds().vxMetersPerSecond < .8){
						addFault("[System Check] Forward speed did not reah target speed in time.", false,true);
					}
				}))
        .until(
            () ->
                !getFaults().isEmpty())
        .andThen(runOnce(() ->setChassisSpeeds(new ChassisSpeeds(0,0,0))));
	}
	private boolean collisionDetected() {
		double curr_world_linear_accel_x = gyro.getWorldLinearAccelX();
		double currentJerkX = curr_world_linear_accel_x
				- last_world_linear_accel_x;
		last_world_linear_accel_x = curr_world_linear_accel_x;
		double curr_world_linear_accel_y = gyro.getWorldLinearAccelY();
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
	public HashMap<String, Double> getTemps() {
		 HashMap<String, Double> tempMap = new HashMap<>();
		 tempMap.put("FLTemp", sparkMotors[0].getMotorTemperature());
		 tempMap.put("FRTemp", sparkMotors[1].getMotorTemperature());
		 tempMap.put("BLTemp", sparkMotors[2].getMotorTemperature());
		 tempMap.put("BRTemp", sparkMotors[3].getMotorTemperature());
		 return tempMap;
	}	
}
