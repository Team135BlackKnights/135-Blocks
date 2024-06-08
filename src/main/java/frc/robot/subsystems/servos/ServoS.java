package frc.robot.subsystems.servos;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.servos.ServoSim;


import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;
import frc.robot.Constants;
import frc.robot.utils.servos.ServoConstantContainer;

public class ServoS extends SubsystemBase{
	public Servo servo;
	public ServoSim servoSim;
	public ServoS(){
				servo = new Servo(ServoConstantContainer.servoPWMPort);
				servoSim = new ServoSim(SimServoMode.INRANGE, ServoType.REVSmartServo, 0, .02);
				servoSim.setSimBounds(-90, 90);
				
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
	@Override
	public void periodic(){
		servoSim.updateServoSim();
	}

}
