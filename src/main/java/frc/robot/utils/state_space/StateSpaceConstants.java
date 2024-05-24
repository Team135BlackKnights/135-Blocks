package frc.robot.utils.state_space;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.MotorConstantContainer;

public class StateSpaceConstants {
	public static boolean debug = true;

	public class Controls {
		/* Enter any non-button controls here.
		 * Left trigger is used for RPM speed. 0-0 1-7100.
		 * Right trigger is used for arm Speed 0-0 1-maxArmSpeed.
		 */
		public static double kDeadband = 0.1, kArmDeadband = 0.1,
				armMoveSpeed = .01, elevatorMoveSpeed = 1;
		public static JoystickButton setButton = new JoystickButton(
				RobotContainer.manipController, 3), //x
				go45Button = new JoystickButton(RobotContainer.manipController, 1), //a
				go0Button = new JoystickButton(RobotContainer.manipController, 2), //b
				go2ftButton = new JoystickButton(RobotContainer.manipController, 3), //x
				go0ftButton = new JoystickButton(RobotContainer.manipController, 4); //y
	}

	public class Flywheel {
		public static boolean inverted = false;
		public static IdleMode mode = IdleMode.kBrake;
		public static int kMotorID = 20, maxRPM = 7100;
		public static MotorConstantContainer flywheelValueHolder = new MotorConstantContainer(
				-0.089838, 0.0015425 * .928, 0.00039717 * 1, 0, 0);
		public static double m_KalmanModel = 3, m_KalmanEncoder = 0.01,
				m_LQRQelms = 1, m_LQRRVolts = 12, flywheelGearing = 1.5;
	}

	public class Arm {
		public static boolean inverted = false;
		public static IdleMode mode = IdleMode.kBrake;
		public static int kMotorID = 30;
		public static MotorConstantContainer armValueHolder = new MotorConstantContainer(
				.001, .001, .001, 0, 0); //must have position set in SysId
		public static double m_KalmanModelPosition = .015,
				m_KalmanModelVelocity = .17, m_KalmanEncoder = 0.01,
				m_LQRQelmsPosition = 1, m_LQRQelmsVelocity = 10, m_LQRRVolts = 12,
				armGearing = 1.5, maxSpeed = DCMotor.getNEO(1).freeSpeedRadPerSec,
				maxAcceleration = DCMotor.getNEO(1).freeSpeedRadPerSec / 2,
				startingPosition = Units.degreesToRadians(0),
				maxPosition = Units.degreesToRadians(90),
				armLength = Units.inchesToMeters(5),
				physicalX = Units.inchesToMeters(20),
				physicalY = Units.inchesToMeters(DriveConstants.kChassisWidth / 2),
				simX = Units.inchesToMeters(11.5),
				simY = Units.inchesToMeters(0),
				simZ = Units.inchesToMeters(4.82);
	}
	public class Elevator {
		public static boolean inverted = false;
		public static IdleMode mode = IdleMode.kBrake;
		public static int kMotorID = 40;
		public static MotorConstantContainer elevatorValueHolder = new MotorConstantContainer(
				.001, .001, .001, 0, 0); //must have position set in SysId
		public static double m_KalmanModelPosition = Units.inchesToMeters(1),
				m_KalmanModelVelocity = Units.inchesToMeters(40), m_KalmanEncoder = 0.001,
				m_LQRQelmsPosition = 1, m_LQRQelmsVelocity = 10, m_LQRRVolts = 12,
				elevatorGearing = 1.5, carriageMass = Units.lbsToKilograms(10), drumRadius = Units.inchesToMeters(.75),
				maxSpeed = Units.feetToMeters(3),
				maxAcceleration = Units.feetToMeters(6),
				startingPosition = Units.inchesToMeters(0),
				maxPosition = Units.feetToMeters(3),
				armLength = Units.inchesToMeters(5),
				physicalX = Units.inchesToMeters(20),
				physicalY = Units.inchesToMeters(DriveConstants.kChassisWidth / 2),
				simX = Units.inchesToMeters(5),
				simY = Units.inchesToMeters(5),
				simZ = Units.inchesToMeters(5);
	}
}
