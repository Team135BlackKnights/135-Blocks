package frc.robot.commands.autos;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class WEDONE extends Command{
	private boolean isFinished = false;
	private Timer timer = new Timer();
	public WEDONE(){

	}
	@Override
	public void initialize(){
		System.err.println("ROBOT COMIN");
		timer.reset();
		timer.start();
		isFinished = false;
	}
	@Override
	public void execute(){
		if (timer.get() > 1.5){
			isFinished = true;
		}
	}
	@Override
	public void end(boolean interrupted){
		System.err.println("ENDING EARLY!");
		RobotContainer.currentNote = 1;
		timer.stop();

	}
	@Override
	public boolean isFinished(){
		return isFinished;
	}
}
