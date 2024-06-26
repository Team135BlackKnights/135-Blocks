package frc.robot.subsystems.drive.OldREVSwerve;

import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;
import edu.wpi.first.math.geometry.Translation2d;

public class REVModuleConstantContainer {
	private int[] moduleConstantContainerInts;
	private boolean[] moduleConstantContainerBools;
	private double[] moduleConstantContainerDoubles;
	private ModulePosition m_modulePosition;
	private MotorConstantContainer[] moduleMotorConstants;
	private Translation2d moduleTranslation;

	/**
	 * Wrapper class designed to hold all the constants for a rev swerve module
	 * in the 135-blocks framework
	 * 
	 * @param driveMotorID                  The CAN ID of the drive motor
	 * @param turningMotorID                The CAN ID of the turning motor
	 * @param driveMotorReversed            Whether the drive motor is reversed
	 *                                         (positive percent/voltage makes it
	 *                                         go backwards)
	 * @param turningMotorReversed          Whether the turning motor is reversed
	 *                                         (positive percent/voltage makes it
	 *                                         go backwards)
	 * @param absoluteEncoderOffset         The offset of the absolute encoder
	 *                                         (when the module is zeroed, what
	 *                                         value is output)
	 * @param maxModuleSpeed                The maximum speed of the module
	 *                                         (should be the same as the max
	 *                                         speed of the drivetrain )
	 * @param driveGearing                  The gearing of the drive motor (>1 is
	 *                                         a reduction)
	 * @param turnGearing                   The gearing of the turning motor (>1
	 *                                         is a reduction)
	 * @param absoluteEncoderReversed       Whether the absolute encoder is
	 *                                         reversed
	 * @param modulePosition                The position of the module relative
	 *                                         to the center of the robot
	 * @param driveMotorConstantContainer   Motor constant container for the
	 *                                         drive motor
	 * @param turningMotorConstantContainer Motor constant container for the
	 *                                         turning motor
	 * @param moduleTranslation2d           The translation of the module from
	 *                                         the center
	 * @see MotorConstantContainer
	 */
	public REVModuleConstantContainer(int driveMotorID, int turningMotorID,
			boolean driveMotorReversed, boolean turningMotorReversed,
			double absoluteEncoderOffset, double maxModuleSpeed,
			double driveGearing, double turnGearing,
			boolean absoluteEncoderReversed, ModulePosition modulePosition,
			MotorConstantContainer driveMotorConstantContainer,
			MotorConstantContainer turningMotorConstantContainer,
			Translation2d moduleTranslation2d) {
		moduleConstantContainerInts = new int[] { driveMotorID, turningMotorID
		};
		moduleConstantContainerBools = new boolean[] { driveMotorReversed,
				turningMotorReversed, absoluteEncoderReversed
		};
		moduleMotorConstants = new MotorConstantContainer[] {
				driveMotorConstantContainer, turningMotorConstantContainer
		};
		moduleConstantContainerDoubles = new double[] { absoluteEncoderOffset,
				maxModuleSpeed, driveGearing, turnGearing
		};
		m_modulePosition = modulePosition;
		moduleTranslation = moduleTranslation2d;
	}

	/**
	 * @return The drive motor's CAN ID
	 */
	public int getDriveMotorID() { return moduleConstantContainerInts[0]; }

	public int getTurningMotorID() { return moduleConstantContainerInts[1]; }

	/**
	 * @return Whether the drive motor is reversed
	 */
	public boolean getDriveMotorReversed() {
		return moduleConstantContainerBools[0];
	}

	/**
	 * @return Whether the turning motor is reversed
	 */
	public boolean getTurningMotorReversed() {
		return moduleConstantContainerBools[1];
	}

	/**
	 * @return Whether the absolute encoder on the turning module is reversed
	 */
	public boolean getAbsoluteEncoderReversed() {
		return moduleConstantContainerBools[2];
	}

	/**
	 * @return The MotorConstantContainer of the drive motors
	 * @see MotorConstantContainer
	 */
	public MotorConstantContainer getDriveMotorConstantContainer() {
		return moduleMotorConstants[0];
	}

	/**
	 * @return The MotorConstantContainer of the turning motors
	 * @see MotorConstantContainer
	 */
	public MotorConstantContainer getTurningMotorConstantContainer() {
		return moduleMotorConstants[1];
	}

	/**
	 * @return The offset of the absolute encoder
	 */
	public double getAbsoluteEncoderOffset() {
		return moduleConstantContainerDoubles[0];
	}

	/**
	 * @return The maximum speed of the module
	 */
	public double getModuleMaxSpeed() {
		return moduleConstantContainerDoubles[1];
	}

	/**
	 * @return the gearing of the module drive motor
	 */
	public double getDriveMotorGearing() {
		return moduleConstantContainerDoubles[2];
	}

	public double getTurnMotorGearing() {
		return moduleConstantContainerDoubles[3];
	}

	/**
	 * @return The position of the module
	 */
	public ModulePosition getModulePosition() { return m_modulePosition; }

	/**
	 * @return The translation of the module from the center of the robot
	 */
	public Translation2d getTranslation2d() { return moduleTranslation; }
}
