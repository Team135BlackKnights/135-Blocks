package frc.robot.commands.leds;

import frc.robot.subsystems.leds.LEDs;
import frc.robot.utils.leds.LEDConstants;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDRainbowC extends Command {
	//Declaring Variables
	private static double m_rainbowFirstPixelHue;
	private boolean isFinished;
	LEDs ledSubsystemLocal;

	/**
	 * The rainbow function for the code, just an implementation of the default
	 * wpilib addressableLED example
	 */
	public LEDRainbowC(LEDs ledSubsystem) {
		this.ledSubsystemLocal = ledSubsystem;
		addRequirements(ledSubsystemLocal);
	}

	@Override
	public void initialize() {
		m_rainbowFirstPixelHue = 0;
		isFinished = false;
	}

	@Override
	public void execute() {
		// For every pixel
		for (var i = 0; i < LEDs.ledBuffer.getLength(); i++) {
			// shape is a circle so only one value needs to precess
			final var hue = (m_rainbowFirstPixelHue
					+ (i * 180 / LEDConstants.ledBufferLength)) % 180;
			// Set the value (also turn the hue into an integer to make it work)
			LEDs.ledBuffer.setHSV(i, (int) Math.floor(hue), 255, 128);
		}
		// Increase by 3 to make the rainbow "move"
		m_rainbowFirstPixelHue += 3;
		// Check bounds
		m_rainbowFirstPixelHue %= 180;
		LEDs.leds.setData(LEDs.ledBuffer);
	}

	@Override
	public void end(boolean interrupted) { isFinished = true; }

	@Override
	public boolean isFinished() { return isFinished; }
}
