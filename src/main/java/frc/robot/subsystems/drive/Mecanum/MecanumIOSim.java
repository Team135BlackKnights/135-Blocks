// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Mecanum;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.utils.drive.DriveConstants;

public class MecanumIOSim implements MecanumIO {
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private DCMotorSim frontLeft = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private DCMotorSim frontRight = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private DCMotorSim backLeft = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private DCMotorSim backRight = new DCMotorSim(DCMotor.getNEO(1), DriveConstants.TrainConstants.kDriveMotorGearRatio, .01);
	private double frontLeftAppliedVolts = 0.0;
	private double frontRightAppliedVolts = 0.0;
	private double backLeftAppliedVolts = 0.0;
	private double backRightAppliedVolts = 0.0;
	private boolean closedLoop = false;
	private PIDController frontLeftPID = new PIDController(KP, 0.0, KD);
	private PIDController frontRightPID = new PIDController(KP, 0.0, KD);
	private PIDController backLeftPID = new PIDController(KP, 0.0, KD);
	private PIDController backRightPID = new PIDController(KP, 0.0, KD);
	private double frontLeftFFVolts = 0.0;
	private double frontRightFFVolts = 0.0;
	private double backLeftFFVolts = 0.0;
	private double backRightFFVolts = 0.0;
	private final Pigeon2 pigeon = new Pigeon2(30);

	@Override
	public void updateInputs(MecanumIOInputs inputs) {
		if (closedLoop) {
			frontLeftAppliedVolts = MathUtil.clamp(frontLeftPID.calculate(
					frontLeft.getAngularVelocityRadPerSec())
					+ frontLeftFFVolts, -12.0, 12.0);
			frontRightAppliedVolts = MathUtil.clamp(frontRightPID.calculate(
					frontRight.getAngularVelocityRadPerSec())
					+ frontRightFFVolts, -12.0, 12.0);
			backLeftAppliedVolts = MathUtil.clamp(backLeftPID.calculate(
					backLeft.getAngularVelocityRadPerSec())
						+ backLeftFFVolts, -12.0, 12.0);
			backRightAppliedVolts = MathUtil.clamp(backRightPID.calculate(
					backRight.getAngularVelocityRadPerSec())
						+ backRightFFVolts, -12.0, 12.0);
			frontLeft.setInputVoltage(frontLeftAppliedVolts);
			frontRight.setInputVoltage(frontRightAppliedVolts);
			backLeft.setInputVoltage(backLeftAppliedVolts);
			backRight.setInputVoltage(backRightAppliedVolts);
		}
		frontLeft.update(.02);
		frontRight.update(.02);
		backLeft.update(.02);
		backRight.update(.02);
		    // Update gyro simulation (you might want to base this on your robot's movement)
		//Pigeon2SimState simState = pigeon.getSimState();
		//double angularVelocity = (frontLeft.getAngularVelocityRadPerSec() - frontRight.getAngularVelocityRadPerSec()
		//	 + backLeft.getAngularVelocityRadPerSec() - backRight.getAngularVelocityRadPerSec()) / 4.0;
		//simState.addYaw(Units.radiansToDegrees(angularVelocity));
		inputs.leftFrontPositionRad = frontLeft.getAngularPositionRad();
		inputs.leftFrontVelocityRadPerSec = frontLeft.getAngularVelocityRadPerSec() ;
		inputs.leftFrontAppliedVolts = frontLeftAppliedVolts;
		inputs.leftBackPositionRad = backLeft.getAngularPositionRad();
		inputs.leftBackVelocityRadPerSec = backLeft.getAngularVelocityRadPerSec();
		inputs.leftBackAppliedVolts = backLeftAppliedVolts;
		inputs.leftCurrentAmps = new double[] { frontLeft.getCurrentDrawAmps(), backLeft.getCurrentDrawAmps()
		};
		inputs.rightFrontPositionRad = frontRight.getAngularPositionRad();
		inputs.rightFrontVelocityRadPerSec = frontRight.getAngularVelocityRadPerSec();
		inputs.rightFrontAppliedVolts = frontRightAppliedVolts;
		inputs.rightBackPositionRad = backRight.getAngularPositionRad();
		inputs.rightBackVelocityRadPerSec = backRight.getAngularVelocityRadPerSec();
		inputs.rightBackAppliedVolts = backRightAppliedVolts;
		inputs.rightCurrentAmps = new double[] { frontRight.getCurrentDrawAmps(), backRight.getCurrentDrawAmps()
		};
		inputs.gyroConnected = false;
		inputs.gyroYaw = pigeon.getRotation2d();
	}

	@Override
	public void setVoltage(double frontLeftVolts, double frontRightVolts,double backLeftVolts, double backRightVolts) {
		closedLoop = false;
		frontLeftAppliedVolts = MathUtil.clamp(frontLeftVolts, -12.0, 12.0);
		frontRightAppliedVolts = MathUtil.clamp(frontRightVolts, -12.0, 12.0);
		backLeftAppliedVolts = MathUtil.clamp(backLeftVolts, -12.0, 12.0);
		backRightAppliedVolts = MathUtil.clamp(backRightVolts, -12.0, 12.0);
		frontLeft.setInputVoltage(frontLeftVolts);
		frontRight.setInputVoltage(frontRightVolts);
		backLeft.setInputVoltage(backLeftVolts);
		backRight.setInputVoltage(backRightVolts);
	}

	@Override
	public void setVelocity(double frontLeftRadPerSec,
	double frontRightRadPerSec, double backLeftRadPerSec,
	double backRightRadPerSec, double frontLeftFFVolts,
	double frontRightFFVolts, double backLeftFFVolts,
	double backRightFFVolts) {
		closedLoop = true;
		frontLeftPID.setSetpoint(frontLeftRadPerSec);
		frontRightPID.setSetpoint(frontRightRadPerSec);
		backLeftPID.setSetpoint(backLeftRadPerSec);
		backRightPID.setSetpoint(backRightRadPerSec);
		this.frontLeftFFVolts = frontLeftFFVolts;
		this.frontRightFFVolts = frontRightFFVolts;
		this.backLeftFFVolts = backLeftFFVolts;
		this.backRightFFVolts = backRightFFVolts;
	}
}
