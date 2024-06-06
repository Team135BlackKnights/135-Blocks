package frc.robot.subsystems.drive.servos;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.servos.ServoSim;


import frc.robot.utils.servos.ServoConstantContainer.ServoType;
import frc.robot.utils.servos.ServoConstantContainer.SimServoMode;
import frc.robot.utils.servos.ServoConstantContainer;

public class ServoS extends SubsystemBase{
	public Servo servo;
	public ServoSim servoSim;
	public ServoS(){
				servo = new Servo(ServoConstantContainer.servoPWMPort);
				servoSim = new ServoSim(SimServoMode.CONTINUOUS, ServoType.REVSmartServo, 1, 0);
				servoSim.setSimBounds(-90, 90);
	}
	public void setServoDegrees(double degrees){
		servo.set(degrees);
	}
	@Override
	public void periodic(){
		servoSim.updateServoSim(.02);
	}

}
