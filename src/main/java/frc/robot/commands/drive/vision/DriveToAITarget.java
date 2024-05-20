package frc.robot.commands.drive.vision;


import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.vision.PhotonVisionS;
import frc.robot.subsystems.drive.SwerveS;
import frc.robot.utils.SimGamePiece;
import frc.robot.utils.drive.DriveConstants;

public class DriveToAITarget extends Command {
	private final SwerveS swerveS;
	private boolean isFinished = false;
	private boolean loaded = false;
	public static boolean allClear = false;
	public static boolean takeOver = false;
	private static boolean close = false;
	private Translation2d targetPieceLocation = null;
	private Pose2d currentPose;
	private ChassisSpeeds speeds;
	double ty;
	Timer timer = new Timer();
	Timer delayTimer = new Timer();
	double desiredHeading, currentHeading, error;

	/*
	 * Call this in all cases but
	 * simulation autonomous
	 */
	public DriveToAITarget(SwerveS swerveS, BooleanSupplier piecePickedUp) {
		this.swerveS = swerveS;

	}

	/*
	 * Call this for simulation
	 * autonomous only
	 */
	public DriveToAITarget(BooleanSupplier piecePickedUp, SwerveS swerveS,
			Translation2d fieldPiecePose) {
		this.swerveS = swerveS;
		this.targetPieceLocation = fieldPiecePose;
	}

	@Override
	public void initialize() {
		if (Constants.currentMode == Mode.SIM) {
			//If the robot is in sim, target the closest game piece to drive to
			this.targetPieceLocation = SimGamePiece.getClosestGamePiece();
		}
		isFinished = false;
		LimelightHelpers.setPipelineIndex(
				VisionConstants.limelightName, 1);
		delayTimer.reset();
		allClear = false;
		close = false;
		takeOver = true;
		ty = 0;
	}

	

	@Override
	public void execute() {

		double tx, ty, distance = 0;
		boolean tv;
		if (Constants.currentMode == Mode.SIM) {
			//Computing distance
			currentPose = SwerveS.getPose();
			//THESE ARE IN M E T E R S 
			tx = targetPieceLocation.getX()
					- currentPose.getX();// -0.307975;
			ty = targetPieceLocation.getY()
					- currentPose.getY();// -0.307975;
			distance = Math
					.sqrt(Math.pow(tx, 2) + Math.pow(ty, 2));
			distance -= DriveConstants.kChassisLength;
			distance = Units.metersToInches(distance);
			tv = true;
			// Calculate the angle to aim towards the target
			double angleToTarget;
			if (tx == 0) {
				angleToTarget = 90;
			} else {
				angleToTarget = Math.atan2(ty, tx); // Calculate angle in radians
				angleToTarget = Math.toDegrees(angleToTarget);
			}
			// Convert angle to degrees if necessary
			// Adjust the robot's orientation towards the target
			// Example: Set the desired heading for your motion control system
			desiredHeading = angleToTarget;
			currentHeading = SwerveS.getHeading();
			error = -1 * (currentHeading - desiredHeading);
			error = PhotonVisionS.closerAngleToZero(error);
		} else {
			//when done, set timer.start().. and delayTimer.stop();
			//THESE ARE IN D E G R E E S 
			tx = LimelightHelpers.getTX(
					VisionConstants.limelightName);
			tv = LimelightHelpers.getTV(
					VisionConstants.limelightName);
			ty = LimelightHelpers.getTY(
					VisionConstants.limelightName);
			error = -tx; //tx is neg apparently -docs
			distance = PhotonVisionS.calculateDistanceFromtY(ty);
		}
		SmartDashboard.putNumber("ANGLE ERROR", error);
		SmartDashboard.putNumber("DISTANCE", distance);
		// SmartDashboard.putBoolean("Piece Loaded?", IntakeS.PieceIsLoaded());
		if (Robot.isReal()) {
			if (PhotonVisionS.calculateDistanceFromtY(ty) <= 4.5) { //less than twelve inches away, tune down!
				close = true;
			}
		} else {
			SmartDashboard.putNumber("SIMDISTANCE ",
					Units.metersToInches(distance));
			if (distance <= 4.5) {
				close = true;
			}
		}
		if (tv == false && loaded == false
				&& close == false) {
			// We don't see the target, seek for the target by spinning in place at a safe speed.
			speeds = new ChassisSpeeds(0, 0, 0.1
					* DriveConstants.kMaxTurningSpeedRadPerSec);
		} else if (loaded == false && close == false) {
			double speedMapperVal = PhotonVisionS.speedMapper(distance);
			double moveSpeed = DriveConstants.kMaxSpeedMetersPerSecond
					* speedMapperVal;
			double trueError = error;
			double turnSpeed = Units.degreesToRadians(trueError)
					* VisionConstants.DriveToAITargetKp * DriveConstants.kMaxTurningSpeedRadPerSec* (2*speedMapperVal);
			speeds = new ChassisSpeeds(moveSpeed, 0,
					turnSpeed);
		} else {
			speeds = new ChassisSpeeds(0, 0, 0);
		}
		swerveS.setChassisSpeeds(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		//Return control of the drivetrain
		takeOver = false;
		timer.stop();
		timer.reset();
		delayTimer.stop();
		delayTimer.reset();
		speeds = new ChassisSpeeds(0, 0, 0);
		swerveS.setChassisSpeeds(speeds);
		if (Robot.isSimulation() && close) {
			if (SimGamePiece.currentPieces.get(0).getZ() > Units.inchesToMeters(1)) {
				SimGamePiece.intake(SimGamePiece.closestPieceIndex - 1);
			} else {
				SimGamePiece.intake(SimGamePiece.closestPieceIndex);
			}
		}
		close = false;
	}

	@Override
	public boolean isFinished() { return isFinished; }
}


