package frc.robot.commands.leds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDs;

public class LEDBreathingC extends Command {
	//Declaring Variables
	private final LEDs ledSubsystem;
	private final int[] color;
	private boolean isFinished;
	private double breathingLoopValue;
	private final double breathingPeriod;
	private int[] breathingLEDStates;
	/**Changes the brightness of all the LEDs to make them look like they are breathing (kind of like the Razer effect) */
	public LEDBreathingC(int[] color, LEDs subsystem, int breathingLoopPeriod){
	
		this.runsWhenDisabled();
		ledSubsystem = subsystem;
		addRequirements(ledSubsystem);
		this.color = color;
		this.breathingPeriod = breathingLoopPeriod;
		breathingLEDStates = new int[breathingLoopPeriod];
	}
	@Override
	public void initialize(){
		isFinished = false;
		breathingLoopValue = 0;
		isFinished = false;
		//This seems complicated, but all it's doing is just taking the sine of the equation, rounding down, and saving it to an array
		for (var i = 0; i < breathingPeriod; i++) {
			breathingLEDStates[i] = (int) Math.floor(Math
					.abs(Math.sin((i * Math.PI / breathingPeriod)))
					* 255);
		}
	}
	@Override
	public void execute(){
		breathingLoopValue += .25;
		breathingLoopValue %= breathingPeriod;
		final int value = breathingLEDStates[(int) Math.floor(breathingLoopValue)];
			for (var i = 0; i < (LEDs.ledBuffer.getLength()); i++) {
				LEDs.ledBuffer.setHSV(i, color[0], color[1], value);
				LEDs.leds.setData(LEDs.ledBuffer);
			}
			//offset by one "notch" each time
			//Prevents any kind of integer overflow error happening (prob would take the robot running for a year straight or something like that to reach, )
			//sets data to buffer
			LEDs.leds.setData(LEDs.ledBuffer);
		}
	@Override
	public void end(boolean isFinished){
		isFinished = true;
	}
	@Override
	public boolean isFinished(){
		return isFinished;
	}
	}
	


