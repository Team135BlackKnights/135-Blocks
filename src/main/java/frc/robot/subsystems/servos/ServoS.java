package frc.robot.subsystems.servos;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.servos.ServoSim;


import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;
import frc.robot.Constants;
import frc.robot.utils.servos.ServoConstantContainer;

public class ServoS extends SubsystemBase{
	private Servo servo;
	private ServoSim servoSim;
	public ServoS(){
				servo = new Servo(ServoConstantContainer.servoPWMPort);
				servoSim = new ServoSim(SimServoMode.INRANGE, ServoType.REVSmartServo, 0, .02);
				servoSim.setSimBounds(ServoConstantContainer.lowerBound, ServoConstantContainer.upperBound);
				
	}
	public void setServoDegrees(double degrees){
		switch (Constants.currentMode) {
			case REAL:
				servo.setAngle(degrees);
				break;

			default:
				servoSim.setAngle(degrees);
				break;
		}
	}
	public void setServoPercent(double percent){
		switch (Constants.currentMode) {
			case REAL:
				servo.set(percent);
				break;
		
			default:
				servoSim.set(percent);
				break;
		}
	}
	public double getServoDegrees(){
		switch (Constants.currentMode) {
			case REAL:
				return ServoConstantContainer.lowerBound + servo.getPosition() * (ServoConstantContainer.upperBound - ServoConstantContainer.lowerBound); 
			default:
				return servoSim.getAngularPositionDegrees();
		}
	}
	@Override
	public void periodic(){
		servoSim.updateServoSim();
	}

}
