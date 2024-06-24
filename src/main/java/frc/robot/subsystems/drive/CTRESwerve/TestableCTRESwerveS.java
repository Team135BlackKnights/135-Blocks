package frc.robot.subsystems.drive.CTRESwerve;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.utils.drive.DriveConstants;
import java.util.List;
import java.util.ArrayList;
import java.util.HashMap;

public class TestableCTRESwerveS extends SubsystemChecker
		implements DrivetrainS {
	final CTRESwerveS ctreSwerveS;
	TalonFX[][] motors;

	public TestableCTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			double OdometryUpdateFrequency, Telemetry logger,
			SwerveModuleConstants... modules) {
		this.ctreSwerveS = new CTRESwerveS(driveTrainConstants,
				OdometryUpdateFrequency, logger, modules);
		motors = makeMotors();
		AutoBuilder.configureHolonomic(() -> getPose(), // Supplier of current robot pose
				ctreSwerveS::seedFieldRelative, // Consumer for seeding pose against auto
				this::getChassisSpeeds, this::setChassisSpeeds, // Consumer of ChassisSpeeds to drive the robot
				new HolonomicPathFollowerConfig(new PIDConstants(8, 0, 0),
						new PIDConstants(8, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
						DriveConstants.kDriveBaseRadius, new ReplanningConfig(true,true)),
				() -> DriverStation.getAlliance()
						.orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
				this);
		registerSelfCheckHardware();
	}

	/**
	 * Creates a CTRE Swerve Drivetrain. Does not have an updateOdometryFrequency
	 * since this is run in sim.
	 * 
	 * @param driveTrainConstants Example of this is in TunerConstants.java
	 * @param logger              //Where to log the data
	 * @param modules             //Which modules to use
	 */
	public TestableCTRESwerveS(SwerveDrivetrainConstants driveTrainConstants,
			Telemetry logger, SwerveModuleConstants... modules) {
		this.ctreSwerveS = new CTRESwerveS(driveTrainConstants, logger, modules);
		motors = makeMotors();
		//Pathplanner declaration
		AutoBuilder.configureHolonomic(() -> ctreSwerveS.getState().Pose, // Supplier of current robot pose
				ctreSwerveS::seedFieldRelative, // Consumer for seeding pose against auto
				ctreSwerveS::getChassisSpeeds, this::setChassisSpeeds, // Consumer of ChassisSpeeds to drive the robot
				new HolonomicPathFollowerConfig(new PIDConstants(8, 0, 0),
						new PIDConstants(8, 0, 0), TunerConstants.kSpeedAt12VoltsMps,
						DriveConstants.kDriveBaseRadius, new ReplanningConfig(true,true)),
				() -> DriverStation.getAlliance()
						.orElse(Alliance.Blue) == Alliance.Red, // Assume the path needs to be flipped for Red vs Blue, this is normally the case
				this);
		registerSelfCheckHardware();
	}

	private TalonFX[][] makeMotors() {
		return new TalonFX[][] { { ctreSwerveS.getModule(0).getDriveMotor(),
				ctreSwerveS.getModule(0).getSteerMotor()
				}, { ctreSwerveS.getModule(1).getDriveMotor(),
						ctreSwerveS.getModule(1).getSteerMotor()
				}, { ctreSwerveS.getModule(2).getDriveMotor(),
						ctreSwerveS.getModule(2).getSteerMotor()
				}, { ctreSwerveS.getModule(3).getDriveMotor(),
						ctreSwerveS.getModule(3).getSteerMotor()
				}
		};
	}
	@Override
	public void periodic(){
		DrivetrainS.super.periodic();
		ctreSwerveS.calculateSkidding();
	}
	public void registerSelfCheckHardware() {
		super.registerHardware("IMU", ctreSwerveS.getPigeon2());
		super.registerHardware("FrontLeftDrive", motors[0][0]);
		super.registerHardware("FrontLeftTurn", motors[0][1]);
		super.registerHardware("FrontRightDrive", motors[1][0]);
		super.registerHardware("FrontRightTurn", motors[1][1]);
		super.registerHardware("BackLeftDrive", motors[2][0]);
		super.registerHardware("BackLeftTurn", motors[2][1]);
		super.registerHardware("BackRightDrive", motors[3][0]);
		super.registerHardware("BackRightTurn", motors[3][1]);
	}

	@Override
	public List<ParentDevice> getOrchestraDevices() {
		List<ParentDevice> orchestra = new ArrayList<>();
		for (var module : motors) {
			orchestra.add(module[0]);
			orchestra.add(module[1]);
		}
		return orchestra;
	}
	@Override
	public SystemStatus getTrueSystemStatus() { return getSystemStatus(); }

	@Override
	public Command getRunnableSystemCheckCommand() {
		return super.getSystemCheckCommand();
	}

	@Override
	public List<ParentDevice> getDriveOrchestraDevices() {
		return getOrchestraDevices();
	}

	@Override
	protected Command systemCheckCommand() {
		return Commands.sequence(
				run(() -> setChassisSpeeds(new ChassisSpeeds(1.5, 0, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					for (int i = 0; i < ctreSwerveS
							.getState().ModuleStates.length; i++) {
						String name = DriveConstants.TrainConstants.ModulePosition
								.values()[i].name();
						SwerveModuleState module = ctreSwerveS
								.getState().ModuleStates[i];
						if (Math.abs(module.speedMetersPerSecond) < 1.3
								|| Math.abs(module.speedMetersPerSecond) > 1.7) {
							addFault(
									"[System Check] Drive motor encoder velocity too slow for "
											+ name + module.speedMetersPerSecond,false,true);
						}
						//angle could be 0, 180, or mod that
						double angle = module.angle.getDegrees()%360;
						if (Math.abs(Math.abs(angle) - 0)  >= 10 && Math.abs(Math.abs(angle) - 180)  >= 10) {
							addFault("[System Check] Turn angle off for " + name
										+ module.angle.getDegrees(),false,true);
						}
					}
				}), run(() -> setChassisSpeeds(new ChassisSpeeds(0, 1.5, 0)))
						.withTimeout(1.0),
				runOnce(() -> {
					for (int i = 0; i < ctreSwerveS
							.getState().ModuleStates.length; i++) {
						String name = DriveConstants.TrainConstants.ModulePosition
								.values()[i].name();
						SwerveModuleState module = ctreSwerveS
								.getState().ModuleStates[i];
						if (Math.abs(module.speedMetersPerSecond) < 1.3
								|| Math.abs(module.speedMetersPerSecond) > 1.7) {
							addFault(
									"[System Check] Drive motor encoder velocity too slow for "
											+ name + module.speedMetersPerSecond,false,true);
						}
						double angle = module.angle.getDegrees()%360;
						if (Math.abs(Math.abs(angle) - 90) >= 10 && Math.abs(Math.abs(angle) - 270) >= 10) { 
							addFault("[System Check] Turn angle off for " + name
										+ module.angle.getDegrees(),false,true);
						}
					}
				}),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, -0.5)))
						.withTimeout(2.0),
				run(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0.5)))
						.withTimeout(2.0))
				.until(() -> !getFaults().isEmpty()).andThen(
						runOnce(() -> setChassisSpeeds(new ChassisSpeeds(0, 0, 0))));
	}

	@Override
	public void setChassisSpeeds(ChassisSpeeds speeds) {
		ctreSwerveS.setChassisSpeeds(speeds);
	}
	@Override
	public boolean[] isSkidding(){
		return ctreSwerveS.isSkidding();
	}
	@Override
	public ChassisSpeeds getChassisSpeeds() {
		return ctreSwerveS.getChassisSpeeds();
	}

	@Override
	public void resetPose(Pose2d pose) { ctreSwerveS.resetPose(pose); }

	@Override
	public void newVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		ctreSwerveS.newVisionMeasurement(pose, timestamp, estStdDevs);
	}

	@Override
	public Pose2d getPose() { return ctreSwerveS.getPose(); }

	@Override
	public void stopModules() { ctreSwerveS.stopModules(); }

	@Override
	public Rotation2d getRotation2d() { return ctreSwerveS.getRotation2d(); }

	@Override
	public double getYawVelocity() { return ctreSwerveS.getYawVelocity(); }

	@Override
	public Twist2d getFieldVelocity() { return ctreSwerveS.getFieldVelocity(); }

	@Override
	public Command sysIdDynamicTurn(Direction direction) {
		return ctreSwerveS.sysIdDynamicTurn(direction);
	}

	@Override
	public Command sysIdQuasistaticTurn(Direction direction) {
		return ctreSwerveS.sysIdQuasistaticTurn(direction);
	}

	@Override
	public Command sysIdDynamicDrive(Direction direction) {
		return ctreSwerveS.sysIdDynamicDrive(direction);
	}

	@Override
	public Command sysIdQuasistaticDrive(Direction direction) {
		return ctreSwerveS.sysIdQuasistaticDrive(direction);
	}

	@Override
	public void zeroHeading() { ctreSwerveS.zeroHeading(); }

	@Override
	public boolean isConnected() { return ctreSwerveS.isConnected(); }

	@Override
	public void applyRequest() { ctreSwerveS.applyRequest(); }

	@Override
	public double getCurrent() {
		return 0;//ctreSwerveS.getCurrent(); //ENABLE IF WANT PROPER CURRENT, BE AWARE CAUSES SOME RESIM'S NEEDED!
	}
	
	@Override
	public HashMap<String, Double> getTemps() {
		 HashMap<String, Double> tempMap = new HashMap<>();
		 tempMap.put("FLDriveTemp", motors[0][0].getDeviceTemp().getValueAsDouble());
		 tempMap.put("FLTurnTemp", motors[0][1].getDeviceTemp().getValueAsDouble());
		 tempMap.put("FRDriveTemp", motors[1][0].getDeviceTemp().getValueAsDouble());
		 tempMap.put("FRTurnTemp", motors[1][1].getDeviceTemp().getValueAsDouble());
		 tempMap.put("BLDriveTemp", motors[2][0].getDeviceTemp().getValueAsDouble());
		 tempMap.put("BLTurnTemp", motors[2][1].getDeviceTemp().getValueAsDouble());
		 tempMap.put("BRDriveTemp", motors[3][0].getDeviceTemp().getValueAsDouble());
		 tempMap.put("BRTurnTemp", motors[3][1].getDeviceTemp().getValueAsDouble());
		 return tempMap;
	}	
}
