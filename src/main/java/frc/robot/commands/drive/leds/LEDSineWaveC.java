package frc.robot.commands.drive.leds;

import frc.robot.subsystems.leds.LEDs;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDSineWaveC extends Command {
	//Declaring Variables
	private final int sinePeriod;
	private int[] color;
	private int[] ledStates ;
	private LEDs ledS;
	private static int initialLoopValue = 0;
	private boolean isFinished;

	/**
	 * Sets the LEDs to one color, but changes the brightness to make a "Wave"
	 * function
	 */
	public LEDSineWaveC(LEDs ledSubsystem, int[] color, int sinePeriodLocal) {
		sinePeriod = sinePeriodLocal;
		this.color = color;
		this.ledS = ledSubsystem;
		ledStates = new int[sinePeriodLocal];
		addRequirements(ledS);
	}

	@Override
	public void initialize() {
		isFinished = false;
		//This seems complicated, but all it's doing is just taking the sine of the equation, rounding down, and saving it to an array
		for (var i = 0; i < sinePeriod; i++) {
			ledStates[i] = (int) Math.floor(Math
					.abs(Math.sin(((i * Math.PI / sinePeriod) + initialLoopValue)))
					* 255);
		}
	}

	@Override
	public void execute() {
		//value is basically how dark it is, is controlled by the wave function
		for (var i = 0; i < (LEDs.ledBuffer.getLength()); i++) {
			final int value = ledStates[(i + initialLoopValue) % sinePeriod];
			LEDs.ledBuffer.setHSV(i, color[0], color[1], value);
		}
		//offset by one "notch" each time
		initialLoopValue += 1;
		//Prevents any kind of integer overflow error happening (prob would take the robot running for a year straight or something like that to reach, )
		initialLoopValue %= sinePeriod;
		//sets data to buffer
		LEDs.leds.setData(LEDs.ledBuffer);
	}

	@Override
	public void end(boolean interrupted) { isFinished = true; }

	@Override
	public boolean isFinished() { return isFinished; }
}
