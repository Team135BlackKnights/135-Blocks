package frc.robot.commands.drive.vision;



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
import frc.robot.Constants.FRCMatchState;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.subsystems.vision.PhotonVisionS;
import frc.robot.utils.SimGamePiece;
import frc.robot.utils.drive.DriveConstants;
/**
 * Implementation of this command is a little different.
 * You must have a way to detect a game piece within the robot, or however you END this command.
 * For an example methodology on this, please refer to the GITHUB 135 Crescendo2024 for IntakeS, and read through the ways we detected game pieces (color sensor because the object was a bright color!)
 * Beyond that, the distance value must be changed within Constants, where that value is in INCHES.
 */
public class DriveToAITarget extends Command {
	private final DrivetrainS swerveS;
	private boolean isFinished = false;
	private boolean loaded = false; //do we have game Piece?
	public static boolean takeOver = false; //to stop driver input
	private static boolean close = false; //when close, stop moving
	private Translation2d targetPieceLocation = null; //no target known unless in SIM
	private Pose2d currentPose;
	private ChassisSpeeds speeds;
	double ty; //pitch
	Timer timer = new Timer();
	double desiredHeading, currentHeading, error;

	/*
	 * Call this in all cases but
	 * simulation autonomous
	 */
	public DriveToAITarget(DrivetrainS swerveS) {
		this.swerveS = swerveS;

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
		timer.reset();
		timer.start();
		close = false;
		takeOver = true; //stop user control
		ty = 0;
	}

	

	@Override
	public void execute() {

		double tx, ty, distance = 0;
		boolean tv;
		if (Constants.currentMode == Mode.SIM) {
			//In sim, we simply grab the 
			//Computing distance
			currentPose = swerveS.getPose();
			//THESE ARE IN M E T E R S 
			tx = targetPieceLocation.getX()
					- currentPose.getX(); 
			ty = targetPieceLocation.getY()
					- currentPose.getY();
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
			currentHeading = swerveS.getRotation2d().getDegrees();
			error = -1 * (currentHeading - desiredHeading);
			error = PhotonVisionS.closerAngleToZero(error);
		} else {
			//THESE ARE IN D E G R E E S 
			tx = LimelightHelpers.getTX(
					VisionConstants.limelightName);
			tv = LimelightHelpers.getTV(
					VisionConstants.limelightName);
			ty = LimelightHelpers.getTY(
					VisionConstants.limelightName);
			error = -tx; //tx is negative
			distance = PhotonVisionS.calculateDistanceFromtY(ty); //returns inches
		}
		if (VisionConstants.debug){
			SmartDashboard.putNumber("ANGLE ERROR", error);
			SmartDashboard.putNumber("DISTANCE", distance);
		}
		// SmartDashboard.putBoolean("Piece Loaded?", IntakeS.PieceIsLoaded());
		if (Constants.currentMode == Mode.REAL) {
			if (PhotonVisionS.calculateDistanceFromtY(ty) <= 4.5) { //less than 4.5 inches away, STOP!
				close = true;
			}
			if (Constants.currentMatchState == FRCMatchState.AUTO && timer.get() > VisionConstants.DriveToAIMaxAutoTime){
				isFinished = true; //if in auto, and greater than max time, STOP ENTIRE COMMAND
			}
		} else {
			if (distance <= 4.5) {
				close = true;
			}
		}
		if (tv == false && loaded == false
				&& close == false) {
			// We don't see the target, seek for the target by spinning in place at a safe speed.
			speeds = new ChassisSpeeds(0, 0, 0.1
					* DriveConstants.kMaxTurningSpeedRadPerSec);
		} else if (loaded == false && close == false) { //We see a target, and we're not close.
			double speedMapperVal = PhotonVisionS.speedMapper(distance); //change speed based on distance
			double moveSpeed = DriveConstants.kMaxSpeedMetersPerSecond
					* speedMapperVal;
			double trueError = error; //Add/subtract from this for any tweaking from where camera placed for actual robot error
			double turnSpeed = Units.degreesToRadians(trueError)
					* VisionConstants.DriveToAITargetKp * DriveConstants.kMaxTurningSpeedRadPerSec* (2*speedMapperVal);//Make turning relative to distance, and a Kp
			speeds = new ChassisSpeeds(moveSpeed, 0,
					turnSpeed);
		} else {
			speeds = new ChassisSpeeds(0, 0, 0); //We either have it, or are close enough to start just intaking, so stop.
		}
		if (loaded){
			isFinished = true;
		}
		swerveS.setChassisSpeeds(speeds);
	}

	@Override
	public void end(boolean interrupted) {
		//Return control of the drivetrain
		takeOver = false;
		timer.stop();
		timer.reset();
		speeds = new ChassisSpeeds(0, 0, 0);
		swerveS.setChassisSpeeds(speeds);
		if (Constants.currentMode == Mode.SIM && close) {
			if (SimGamePiece.currentPieces.get(0).getZ() > Units.inchesToMeters(1)) { //if another game piece is off the ground, properly update the simArray
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

