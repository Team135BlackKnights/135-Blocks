package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import java.util.Set;
import frc.robot.Constants;
import frc.robot.Constants.FRCMatchState;

import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;

// Heads up! It's unimportant, but if the driver shoots a Piece in sim, picks up
// a Piece BEFORE the shot one is at the amp, and then attempts to shoot the new
// Piece BEFORE the old shot one is at the amp, it will desync
// the array by 1, which will crash the program at the last Piece on the field.
// I'm not fixing this, because it'd require a lot of time, and is a VERY
// one-off scenario.
public class SimGamePiece {
	//Speaker translations
	private static final Translation3d blueShootLocation = new Translation3d(0.225,5.55, 2.1);
	private static final Translation3d redShootLocation = new Translation3d(16.317,5.55, 2.1); //in meters!
	private static final double shotSpeed = 15;
	private static final double intakeSpeed = 3;
	private static Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
	//Launcher position compared to the robot
	static Transform3d launcherTransform = new Transform3d(0.292, 0, 0.1225,
			new Rotation3d(0, 0, 0.0));

	//Returns the robot pose
	public static void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
		robotPoseSupplier = supplier;
	}

	public static ArrayList<Pose3d> currentPieces = new ArrayList<Pose3d>();
	static Pose3d heldPiecePos = new Pose3d(robotPoseSupplier.get()).transformBy(launcherTransform);
	static public boolean firstIntakeCycle = true, hasPiece = false, firstOutake = true;

	public static void resesPieces() {
		currentPieces.clear();
		for (int i = 0; i < Constants.DriveSimConstants.fieldPieceTranslations.length; i++) {
			currentPieces.add(Constants.DriveSimConstants.fieldPieceTranslations[i]);
		}
		if (Constants.currentMatchState == FRCMatchState.AUTO) {
			currentPieces.add(0, heldPiecePos);
			hasPiece = true;
		}
	}

	public static void intake(int index) {
		new ScheduleCommand( // Branch off and exit immediately
				Commands.defer(() -> {
					launcherTransform = new Transform3d(0.3, 0, 0.36,
							new Rotation3d(0,
									0,
									0.0));
					final Pose3d startPose = currentPieces.get(index);
					final Pose3d endPose = new Pose3d(robotPoseSupplier.get())
							.transformBy(launcherTransform);
					final double duration = startPose.getTranslation()
							.getDistance(endPose.getTranslation()) / intakeSpeed;
					final Timer timer = new Timer();
					timer.start();
					firstIntakeCycle = true;
					if (!hasPiece) {
						currentPieces.remove(index);
						return Commands.run(() -> {
							if (SimGamePiece.firstIntakeCycle) {
								currentPieces.add(0, startPose.interpolate(endPose, timer.get() / duration));
								SimGamePiece.firstIntakeCycle = false;
							} else {
								currentPieces.remove(0);
								currentPieces.add(0, startPose.interpolate(endPose, timer.get() / duration));
							}
						}).until(() -> timer.hasElapsed(duration)).finallyDo(() -> {
							//currentPieces.remove(0);
							hasPiece = true;
						});
					} else {
						return new InstantCommand();
					}
				}, Set.of()).ignoringDisable(true)).schedule();
	}

	public static Pose3d[] getState() {
		Pose3d[] translator = new Pose3d[currentPieces.size()];
		for (int i = 0; i < currentPieces.size(); i++) {
			translator[i] = currentPieces.get(i);
		}
		return translator;
	}

	public static void updateStates() {
		Pose3d[] translator = new Pose3d[currentPieces.size()];
		launcherTransform = new Transform3d(0.3, 0, 0.36,
				new Rotation3d(0, 0,
				 0.0));
		heldPiecePos = new Pose3d(robotPoseSupplier.get())
				.transformBy(launcherTransform);
		for (int i = 0; i < currentPieces.size(); i++) {
			if (i == 0 && hasPiece) {
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
					final boolean isRed = DriverStation.getAlliance().isPresent()
							&& DriverStation.getAlliance().get().equals(Alliance.Red);
					final Pose3d endPose = new Pose3d(
							isRed ? redShootLocation : blueShootLocation, startPose.getRotation());
					final double duration = startPose.getTranslation()
							.getDistance(endPose.getTranslation()) / shotSpeed;
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
						return Commands.run(() -> 
						{
							if (SimGamePiece.firstOutake && indexChanger == 2) {
								currentPieces.remove(0);
								currentPieces.add(currentPieces.size() - 1, startPose
										.interpolate(endPose, timer.get() / duration));
								SimGamePiece.firstOutake = false;
							} else {
								currentPieces.remove(currentPieces.size() - indexChanger);
								if (indexChanger == 2) {
									currentPieces.add(currentPieces.size() - 1, startPose
											.interpolate(endPose, timer.get() / duration));
								} else {
									currentPieces.add(currentPieces.size(), startPose
											.interpolate(endPose, timer.get() / duration));
								}
							}
						}).until(() -> timer.hasElapsed(duration)).finallyDo(() -> 
						{
							currentPieces.remove(currentPieces.size() - indexChanger);
						});
					} else {
						return new InstantCommand();
					}
				}, Set.of()).ignoringDisable(true));
	}
}