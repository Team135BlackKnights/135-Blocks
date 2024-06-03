package frc.robot.subsystems.drive.REVSwerve;
import frc.robot.utils.MotorConstantContainer;
import frc.robot.utils.drive.DriveConstants.TrainConstants.ModulePosition;
import edu.wpi.first.math.geometry.Translation2d;
public class REVModuleConstantContainer {
	private int[] moduleConstantContainerInts;
	private boolean[] moduleConstantContainerBools;
	private double moduleAbsEncOffset;
	private ModulePosition m_modulePosition;
	private MotorConstantContainer[] moduleMotorConstants;
	private Translation2d moduleTranslation;
	/**
	 * Wrapper class designed to hold all the constants for a rev swerve module in the 135-blocks framework
	 * @param driveMotorID the CAN ID of the drive motor
	 * @param turningMotorID the CAN ID of the turning motor
	 * @param driveMotorReversed whether the drive motor is reversed (positive percent/voltage makes it go backwards)
	 * @param turningMotorReversed whether the turning motor is reversed (positive percent/voltage makes it go backwards)
	 * @param absoluteEncoderOffset the offset of the absolute encoder (when the module is zeroed, what value is output)
	 * @param absoluteEncoderReversed whether the absolute encoder is reversed
	 * @param modulePosition the position of the module relative to the center of the robot
	 * @param driveMotorConstantContainer motor constant container for the drive motor @see motorConstantContainer
	 * @param turningMotorConstantContainer motor constant container for the turning motor @see motorConstantContainer
	 * @param moduleTranslation2d the translation of the module from the center
	 */
	public REVModuleConstantContainer(int driveMotorID, int turningMotorID,
			boolean driveMotorReversed, boolean turningMotorReversed,
			double absoluteEncoderOffset,
			boolean absoluteEncoderReversed,
			ModulePosition modulePosition,
			MotorConstantContainer driveMotorConstantContainer,
			MotorConstantContainer turningMotorConstantContainer, Translation2d moduleTranslation2d){
				moduleConstantContainerInts = new int[]{driveMotorID, turningMotorID};
				moduleConstantContainerBools = new boolean[]{driveMotorReversed,turningMotorReversed,absoluteEncoderReversed};
				moduleMotorConstants = new MotorConstantContainer[]{driveMotorConstantContainer,turningMotorConstantContainer};
				moduleAbsEncOffset = absoluteEncoderOffset;
				m_modulePosition = modulePosition;
				moduleTranslation = moduleTranslation2d;
			}
	public int getDriveMotorID(){
		return moduleConstantContainerInts[0];
	}
	public int getTurningMotorID(){
		return moduleConstantContainerInts[1];
	}
	public boolean getDriveMotorReversed(){
		return moduleConstantContainerBools[0];
	}
	public boolean getTurningMotorReversed(){
		return moduleConstantContainerBools[1];
	}
	public boolean getAbsoluteEncoderReversed(){
		return moduleConstantContainerBools[2];
	}
	public MotorConstantContainer getDriveMotorConstantContainer(){
		return moduleMotorConstants[0];
	}
	public MotorConstantContainer getTurningMotorConstantContainer(){
		return moduleMotorConstants[1];
	}
	public double getAbsoluteEncoderOffset() { 
		return moduleAbsEncOffset;
	 }
	public ModulePosition getModulePosition(){
		return m_modulePosition;
	}		
	public Translation2d getTranslation2d(){
		return moduleTranslation;
	}

}
