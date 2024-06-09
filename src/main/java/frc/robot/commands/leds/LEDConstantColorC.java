package frc.robot.commands.leds;

import frc.robot.subsystems.leds.LEDs;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDConstantColorC extends Command {
	//Declaring Variables
	private LEDs ledSubsystemLocal;
	private int[] color;
	private boolean isFinished;

	/** Designed to make all the LEDs a constant color */
	public LEDConstantColorC(int[] colortoSet, LEDs ledSubsytem) {
		color = colortoSet;
		ledSubsystemLocal = ledSubsytem;
		addRequirements(ledSubsystemLocal);
	}

	@Override
	public void initialize() { isFinished = false; }

	@Override
	public void execute() {
		//for each pixel
		for (var i = 0; i < LEDs.ledBuffer.getLength(); i++) {
			// Sets the specified LED to the RGB values for red
			LEDs.ledBuffer.setHSV(i, color[0], color[1], color[2]);
		}
		//sets values
		LEDs.leds.setData(LEDs.ledBuffer);
	}

	@Override
	public void end(boolean interrupted) { isFinished = true; }

	@Override
	public boolean isFinished() { return isFinished; }
}
