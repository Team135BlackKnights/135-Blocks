package frc.robot.commands.drive.servos;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.servos.ServoS;
public class ServoC extends Command{
	private ServoS servoS;
	public ServoC(ServoS servoS){
		this.servoS = servoS;
		addRequirements(this.servoS);
	}
	@Override
	public void initialize(){

	}
	@Override
	public void execute(){
		servoS.servoSim.set(1);
		SmartDashboard.putNumber("sim pos", servoS.servoSim.getAngularPositionDegrees());
		SmartDashboard.putNumber("sim vel", servoS.servoSim.getAngularVelocityDegreesPerSec());
	}
	@Override
	public void end(boolean interrupted){

	}
	@Override
	public boolean isFinished() { return false; }

}
