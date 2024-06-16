package frc.robot.commands.drive.vision;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.vision.PhotonVisionS;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.LoggableTunedNumber;
import java.util.Optional;

import org.photonvision.PhotonCamera;
public class AimToAprilTag extends Command {
	private final DrivetrainS drivetrainS;
	private final int aprilTagID;
	private final PhotonCamera cam;
	private final ProfiledPIDController thetaController = new ProfiledPIDController(
			0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0), .02);
	// Allow live updating via LoggableTunedNumbers
	private static final LoggableTunedNumber thetaKp = new LoggableTunedNumber(
			"AimToApriltag/ThetaKp");
	private static final LoggableTunedNumber thetaKd = new LoggableTunedNumber(
			"AimToApriltag/ThetaKd");
	private static final LoggableTunedNumber thetaMaxVelocitySlow = new LoggableTunedNumber(
			"AimToApriltag/ThetaMaxVelocitySlow");
	private static final LoggableTunedNumber thetaTolerance = new LoggableTunedNumber(
			"AimToApriltag/ThetaTolerance");
	static {
		thetaKp.initDefault(2);
		thetaKd.initDefault(0.0);
		thetaMaxVelocitySlow.initDefault(Units.degreesToRadians(90.0));
		thetaTolerance.initDefault(Units.degreesToRadians(4.0));
	}

	public AimToAprilTag(DrivetrainS drivetrainS, int aprilTagID, PhotonCamera cam) {
		this.drivetrainS = drivetrainS;
		this.aprilTagID = aprilTagID;
		this.cam = cam;
		thetaController.enableContinuousInput(-Math.PI, Math.PI);
	}

	@Override
	public void initialize() {
		RobotContainer.currentPath = "AIMTOAPRILTAG";
		Pose2d currentPose = drivetrainS.getPose();
		thetaController.reset(currentPose.getRotation().getRadians(),
				drivetrainS.getRotation2d().getRadians());
	}

	@Override
	public void execute() {
		if (thetaKp.hasChanged(hashCode()) || thetaKd.hasChanged(hashCode())
				|| thetaMaxVelocitySlow.hasChanged(hashCode())
				|| thetaTolerance.hasChanged(hashCode())) {
			thetaController.setP(thetaKp.get());
			thetaController.setD(thetaKd.get());
			thetaController.setConstraints(new TrapezoidProfile.Constraints(
					thetaMaxVelocitySlow.get(), Double.POSITIVE_INFINITY));
			thetaController.setTolerance(thetaTolerance.get());
		}
		RobotContainer.currentPath = "AIMTOAPRILTAG";
		Optional<Pose3d> aprilTagPose = PhotonVisionS.aprilTagPose3d(cam, aprilTagID);
		System.out.println(aprilTagPose);
		double thetaVelocity;
		if (aprilTagPose.isPresent()){
			Pose2d drivePose = drivetrainS.getPose();
			Translation2d aprilTranslation2d = aprilTagPose.get().toPose2d().getTranslation();
			Pose2d fieldRelativeAprilTagPose = drivePose.transformBy(GeomUtil.translationToTransform(aprilTranslation2d.getX(),aprilTranslation2d.getY()));
			double targetAngle = Units.degreesToRadians(GeomUtil
				.rotationFromCurrentToTarget(drivePose, fieldRelativeAprilTagPose)
				.getRadians());
		Rotation2d currentRotation = drivePose.getRotation();
		RobotContainer.angleOverrider = Optional
				.of(new Rotation2d(targetAngle));
		thetaVelocity = thetaController.getSetpoint().velocity
				+ thetaController.calculate(currentRotation.getRadians(),
						targetAngle); 
		}else{
			thetaVelocity = 0;
			RobotContainer.angleOverrider = Optional.empty();
		}
		if (Constants.currentMatchState == Constants.FRCMatchState.TELEOP){
			RobotContainer.angularSpeed = thetaVelocity;
		}
	}

	@Override
	public void end(boolean interrupted) {
		RobotContainer.currentPath = "";
		RobotContainer.angleOverrider = Optional.empty();
		RobotContainer.angularSpeed = 0;
	}

	@Override
	public boolean isFinished() { return false; }
}
