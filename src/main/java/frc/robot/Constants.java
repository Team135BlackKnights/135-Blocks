// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.HashMap;
import java.util.Map;

import frc.robot.utils.drive.DriveConstants;
/**
 * Anything for the e n t i r e robot goes here. Specific constants go under
 * Utils, and their respective folder for Merge purposes.
 */
public final class Constants {
	public static final Mode currentMode = Mode.SIM;

	public static enum Mode {
		/** Running on a real robot. */
		REAL,
		/** Running a physics simulator. */
		SIM,
		/** Replaying from a log file. */
		REPLAY
	}
	//FRCMatchState of the robot
	public static FRCMatchState currentMatchState = FRCMatchState.DISABLED;

	/**
	 * Allows the robot to utilize switch statements to efficiently figure out
	 * the match period it's in. The current FRCMatchState is stored in
	 * currentMatchState. Use .name() to get the FRCMatchState as a String
	 */
	public static enum FRCMatchState {
		//Very beginning of test mode
		TESTINIT,
		//When test mode is running
		TEST,
		//Very beginning of auto
		AUTOINIT,
		//Throughout auto
		AUTO,
		//Very beginning of Teleop (Until Endgame)
		TELEOPINIT,
		//Throughout Teleop (Until Endgame)
		TELEOP,
		//Very beginning of Endgame (20 seconds left)
		ENDGAMEINIT,
		//Throughout Endgame
		ENDGAME,
		//When robot is disabled (Default State)
		DISABLED,
		//Runs when the match is over (after endgame)
		MATCHOVER
	}
	//put datalog constants IN THE UTIL FOR THAT FILE. 
	public static Map<Integer, String> manCanIdsToNames() {
		HashMap<Integer, String> map = new HashMap<>();
		//this has to be adjusted for every block branch added!
		map.put(DriveConstants.kBackLeftDrivePort, "backLeftDrive");
		map.put(DriveConstants.kBackLeftTurningPort, "backLeftTurn");
		map.put(DriveConstants.kBackRightDrivePort, "backRightDrive");
		map.put(DriveConstants.kBackRightTurningPort, "backRightTurn");
		map.put(DriveConstants.kFrontLeftDrivePort, "frontLeftDrive");
		map.put(DriveConstants.kFrontLeftTurningPort, "frontLeftTurn");
		map.put(DriveConstants.kFrontRightDrivePort, "frontRightDrive");
		map.put(DriveConstants.kFrontRightTurningPort, "frontRightTurn");

		return map;
	}
}