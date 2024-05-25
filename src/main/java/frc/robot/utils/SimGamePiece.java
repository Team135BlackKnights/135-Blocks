package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

// Heads up! It's unimportant, but if the driver shoots a Piece in sim, picks up
// a Piece BEFORE the shot one is at the amp, and then attempts to shoot the new
// Piece BEFORE the old shot one is at the amp, it will desync
// the array by 1, which will crash the program at the last Piece on the field.
// I'm not fixing this, because it'd require a lot of time, and is a VERY
// one-off scenario.
/**
 * Simulate interactions with elements on the field. Automatically puts back all
 * pieces in the event none are left on the field. Must provide translations to
 * shoot locations, if that's being used that year. (blue/redShootLocation) Must
 * provide shoot speed and intake speed in meters, if being used. Must provide
 * launcherTransform, for where the MANIPULATOR brings the game pieces. Found in
 * Constants
 */
public class SimGamePiece {
	private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
	public static int closestPieceIndex;
	//Returns the robot pose
	public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
		robotPoseSupplier = supplier;
	}

	//contains all game pieces
	public static ArrayList<Pose3d> currentPieces = new ArrayList<Pose3d>();
	//contains the currently held Piece pose. If multiple game pieces can be held, this must be rewritten.
	static Pose3d heldPiecePos = new Pose3d(robotPoseSupplier.get())
			.transformBy(Constants.DriveSimConstants.launcherTransform);
	static public boolean firstIntakeCycle = true, hasPiece = false,
			firstOutake = true;

	public static void resetPieces() {
		currentPieces.clear();
		//add all game pieces
		for (int i = 0; i < Constants.DriveSimConstants.fieldPieceTranslations.length; i++) {
			currentPieces
					.add(Constants.DriveSimConstants.fieldPieceTranslations[i]);
		}
		//if allowed a starting game piece, put it in the robot's spot.
		if (Constants.currentMatchState == FRCMatchState.AUTO) {
			currentPieces.add(0, heldPiecePos);
			hasPiece = true;
		}
	}

	public static void intake(int index) {
		new ScheduleCommand( // Branch off and exit immediately, but run this.
				Commands.defer(() -> {
					// get our start pose for the shot, and end pose.
					final Pose3d startPose = currentPieces.get(index);
					final Pose3d endPose = new Pose3d(robotPoseSupplier.get())
							.transformBy(
									Constants.DriveSimConstants.launcherTransform);
					//set our duration via how long the action will take
					final double duration = startPose.getTranslation()
							.getDistance(endPose.getTranslation())
							/ Constants.DriveSimConstants.intakeSpeed;
					final Timer timer = new Timer();
					timer.start();
					//to make it add the new location on first run, then remove the old and put new, use a debounce.
					firstIntakeCycle = true;
					if (!hasPiece) { //confirm not getting two pieces, CHANGE IF ALLOWED
						currentPieces.remove(index);
						return Commands.run(() -> {
							if (SimGamePiece.firstIntakeCycle) {
								currentPieces.add(0, startPose.interpolate(endPose,
										timer.get() / duration)); //put piece first in array.
								SimGamePiece.firstIntakeCycle = false;
							} else {
								currentPieces.remove(0); //remove old piece
								currentPieces.add(0, startPose.interpolate(endPose,
										timer.get() / duration)); //put new piece
							}
						}).until(() -> timer.hasElapsed(duration)).finallyDo(() -> { //until action done
							hasPiece = true; //for other code to read when this is done
						});
					} else {
						return new InstantCommand();
					}
				}, Set.of()).ignoringDisable(true)).schedule(); //run command branch
	}

	/**
	 * Convert the SimGamePiece array to a Pose3d[]
	 * 
	 * @return Pose3d[] of all game pieces currently existing anywhere.
	 */
	public static Pose3d[] getState() {
		Pose3d[] translator = new Pose3d[currentPieces.size()];
		for (int i = 0; i < currentPieces.size(); i++) {
			translator[i] = currentPieces.get(i);
		}
		return translator;
	}

	/**
	 * Needs to be called every perodic by robot. (is default)
	 */
	public static void updateStates() {
		Pose3d[] translator = new Pose3d[currentPieces.size()];
		heldPiecePos = new Pose3d(robotPoseSupplier.get())
				.transformBy(Constants.DriveSimConstants.launcherTransform);
		for (int i = 0; i < currentPieces.size(); i++) { //for all notes
			if (i == 0 && hasPiece) { //if is zero, and we have a note, make that location the held Note relative to bot
				translator[i] = heldPiecePos;
			} else {
				translator[i] = currentPieces.get(i);
			}
		}
		Logger.recordOutput("GamePieceVisualizer", translator);
	}

	public static Command shoot() {
		return new ScheduleCommand( // Branch off and exit immediately
				Commands.defer(() -> {
					//Initial starting point
					final Pose3d startPose = heldPiecePos;
					final boolean isRed = Robot.isRed;
					final Pose3d endPose = new Pose3d(
							isRed ? Constants.DriveSimConstants.redShootLocation
									: Constants.DriveSimConstants.blueShootLocation,
							startPose.getRotation()); //Go to proper shoot location relative to team
					//Below, refer to Intake()
					final double duration = startPose.getTranslation()
							.getDistance(endPose.getTranslation())
							/ Constants.DriveSimConstants.shotSpeed;
					final Timer timer = new Timer();
					timer.start();
					firstOutake = true;
					if (hasPiece) {
						hasPiece = false;
						int indexChanger;
						if (currentPieces.size() == 1) {
							indexChanger = 1;
						} else {
							indexChanger = 2;
						}
						return Commands.run(() -> {
							if (SimGamePiece.firstOutake && indexChanger == 2) {
								currentPieces.remove(0);
								currentPieces.add(currentPieces.size() - 1, startPose
										.interpolate(endPose, timer.get() / duration));
								SimGamePiece.firstOutake = false;
							} else {
								currentPieces
										.remove(currentPieces.size() - indexChanger);
								if (indexChanger == 2) {
									currentPieces.add(currentPieces.size() - 1, startPose
											.interpolate(endPose, timer.get() / duration));
								} else {
									currentPieces.add(currentPieces.size(), startPose
											.interpolate(endPose, timer.get() / duration));
								}
							}
						}).until(() -> timer.hasElapsed(duration)).finallyDo(() -> {
							currentPieces.remove(currentPieces.size() - indexChanger);
						});
					} else {
						return new InstantCommand();
					}
				}, Set.of()).ignoringDisable(true));
	}
	public static Translation2d getClosestGamePiece() {

        Translation2d closestTrans = new Translation2d();
        double closestPoseDistance = 9999; //hopefully something is closer than 9999 meters
        for (int i = 0; i < getState().length; i++) {
            if (getState()[i].getTranslation().toTranslation2d().getDistance(
					RobotContainer.drivetrainS.getPose().getTranslation()) < closestPoseDistance
                    && getState()[i].getZ() < Units.inchesToMeters(1.1)) 
            {
                closestTrans = getState()[i].getTranslation().toTranslation2d();
                closestPieceIndex = i;
                closestPoseDistance = getState()[i].getTranslation().toTranslation2d().getDistance(
                    RobotContainer.drivetrainS.getPose().getTranslation());
            }
        }
        if (closestPoseDistance == 9999) {
            resetPieces();
            return getClosestGamePiece();
        }
        return closestTrans;
    }
}