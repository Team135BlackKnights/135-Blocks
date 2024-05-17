package frc.robot.utils.state_space;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;

public class StateSpaceConstants {
	public static boolean debug = true;

	public class Controls {
		/* Enter any non-button controls here.
		 * Left trigger is used for RPM speed. 0-0 1-7100.
		 */
		public static double kDeadband = 0.1, kArmDeadband = 0.05;
		public static JoystickButton setButton = new JoystickButton(
				RobotContainer.manipController, 3), //x
				go45Button = new JoystickButton(RobotContainer.manipController, 1), //a
				go0Button = new JoystickButton(RobotContainer.manipController, 2); //b
	}

	public class Flywheel {
		public static boolean inverted = false;
		public static IdleMode mode = IdleMode.kBrake;
		public static int kMotorID = 20, maxRPM = 7100;
		public static double kFlywheelP = 0, kFlywheelSVolts = 0,
				kFlywheelVVoltSecondsPerRotation = 0,
				kFlywheelAVoltSecondsSquaredPerRotation = 0, m_KalmanModel = 3,
				m_KalmanEncoder = 0.01, m_LQRQelms = 1, m_LQRRVolts = 12,
				flywheelGearing = 1.5;
	}

	public class Arm {
		public static boolean inverted = false;
		public static IdleMode mode = IdleMode.kBrake;
		public static int kMotorID = 30;
		public static double kArmP = 0, kArmD = 0, //must have position set in SysId
				kArmSVolts = 0, kArmVVoltSecondsPerRotation = 0,
				kArmAVoltSecondsSquaredPerRotation = 0,
				m_KalmanModelPosition = .015, m_KalmanModelVelocity = .17,
				m_KalmanEncoder = 0.01, m_LQRQelmsPosition = 100,
				m_LQRQelmsVelocity = 5, m_LQRRVolts = 12, armGearing = 1.5,
				maxRPM = Units.radiansPerSecondToRotationsPerMinute(
						DCMotor.getNEO(1).freeSpeedRadPerSec),
				startingPosition = Units.degreesToRadians(0),
				maxPosition = Units.degreesToRadians(90),
				armLength = Units.inchesToMeters(5),
				physicalX = Units.inchesToMeters(20),
				physicalY = Units.inchesToMeters(DriveConstants.kChassisWidth / 2);
	}
}
