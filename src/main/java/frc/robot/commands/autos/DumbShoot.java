package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CTRE_state_space.CTREFlywheelS;

public class DumbShoot extends Command{
	final CTREFlywheelS flywheelS;
	final int RPM;
	private boolean isFinished;
	public DumbShoot(CTREFlywheelS flywheelS, int RPM){
		this.flywheelS = flywheelS;
		this.RPM = RPM;
		addRequirements(flywheelS);
	}
	@Override
	public void initialize(){
		isFinished = false;
	}
	@Override
	public void execute(){
		flywheelS.setRPM(RPM);
		//SmartDashboard.putNumber("Flywheel Speed", CTREFlywheelS.getSpeedError());
		if (Math.abs(CTREFlywheelS.getSpeedError()) < 60){
			isFinished = true;
		}
	}
	@Override
	public void end(boolean interrupted){
		flywheelS.setRPM(0);
	}
	@Override
	public boolean isFinished(){
		return isFinished;
	}
}
