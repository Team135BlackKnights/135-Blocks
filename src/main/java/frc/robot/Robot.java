// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import org.littletonrobotics.urcl.URCL;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import frc.robot.Constants.FRCMatchState;
import frc.robot.utils.SimGamePiece;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command m_autonomousCommand;
	private RobotContainer m_robotContainer;
	public static boolean isRed;
	private boolean hasBeenEnabled;
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		//To check for endgame
		//Below may be redudant
		// Instantiate our RobotContainer.  This will perform all our button bindings, and put our
		// autonomous chooser on the dashboard
		Logger.recordMetadata("ProjectName", "The Chef"); // Set a metadata value
		switch (Constants.currentMode) {
		case REAL:
			// Running on a real robot, log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new WPILOGWriter());
			Logger.addDataReceiver(new NT4Publisher());
			break;
		case SIM:
			// Running a physics simulator, log to NT
			Logger.addDataReceiver(new NT4Publisher());
			break;
		case REPLAY:
			// Replaying a log, set up replay source
			setUseTiming(false); // Run as fast as possible
			String logPath = LogFileUtil.findReplayLog();
			Logger.setReplaySource(new WPILOGReader(logPath));
			Logger.addDataReceiver(
					new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
			break;
		}
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog());
		Logger.registerURCL(URCL.startExternal(Constants.manCanIdsToNames()));
		Logger.start();
		m_robotContainer = new RobotContainer();
		DataHandler.startHandler("C:");

	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for
	 * items like diagnostics that you want ran during disabled, autonomous,
	 * teleoperated and test.
	 * <p>
	 * This runs after the mode specific periodic functions, but before
	 * LiveWindow and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		DataHandler.updateHandlerState();
		SmartDashboard.putString("Match State",
				Constants.currentMatchState.name());
		isRed = DriverStation.getAlliance().isPresent()
				? DriverStation.getAlliance().get() == DriverStation.Alliance.Red
				: false;
		// Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
		// commands, running already-scheduled commands, removing finished or interrupted commands,
		// and running subsystem periodic() methods.  This must be called from the robot's periodic
		// block in order for anything in the Command-based framework to work.
		CommandScheduler.getInstance().run();
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		Constants.currentMatchState = FRCMatchState.DISABLED;
		if (DriverStation.getMatchTime() == 0){
			Constants.currentMatchState = FRCMatchState.MATCHOVER;
		}

	}

	@Override
	public void disabledPeriodic() {}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {
		Constants.currentMatchState = FRCMatchState.AUTOINIT;
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		// schedule the autonomous command (example)
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		Constants.currentMatchState = FRCMatchState.AUTO;
	}

	@Override
	public void teleopInit() {
		Constants.currentMatchState = FRCMatchState.TELEOPINIT;
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		/*An FRC teleop period takes 2 minutes and 15 seconds (135 seconds). Endgame occurs during the last 20 seconds.
		Based on this, endgame should initialize at 115 seconds and end at 135 seconds. */
		if (DriverStation.isFMSAttached()){
			if (DriverStation.getMatchTime() > 30) {
				Constants.currentMatchState = FRCMatchState.TELEOP;
			} else if (DriverStation.getMatchTime() == 30) {
				Constants.currentMatchState = FRCMatchState.ENDGAMEINIT;
			} else if (DriverStation.getMatchTime() < 30
					&& DriverStation.getMatchTime() > 0) {
				Constants.currentMatchState = FRCMatchState.ENDGAME;
			}
		}else{
			Constants.currentMatchState = FRCMatchState.TELEOP;
		}
		if(RobotContainer.driveController.getPOV() == 0){
			//System.err.println("UP");
			DataHandler.logData("[4.5,25.4]", "shouldUpdateModel");
		}
		if(RobotContainer.manipController.getAButton()){
			System.out.println("A");
			DataHandler.logData("4.5","modelDistance");
		}
	}

	@Override
	public void testInit() {
		Constants.currentMatchState = FRCMatchState.TESTINIT;
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
		Constants.currentMatchState = FRCMatchState.TEST;
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		/*We don't assign a match state for either simulation function because we most likely would want to test features that occur
		at different game periods like tele and auto in simulation*/
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		SimGamePiece.updateStates(); //update position of gamePieces
		if (Constants.currentMatchState == Constants.FRCMatchState.AUTO && !hasBeenEnabled) {
			SimGamePiece.resetPieces();
			hasBeenEnabled = true;
		} else if (Constants.currentMatchState == Constants.FRCMatchState.DISABLED) {
			hasBeenEnabled = false;
		}
		//DataHandler.updateHandlerState();
	}
}
