// IO implementation creation files are from
// http://github.com/Mechanical-Advantage
// Be sure to understand how it creates the "inputs" variable and edits it!
package frc.robot.subsystems.drive.Tank;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.utils.drive.DriveConstants;

public class TankIOSim implements TankIO {
	private static final double KP = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getP();
	private static final double KD = DriveConstants.TrainConstants.overallDriveMotorConstantContainer
			.getD();
	private DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
			DCMotor.getNEO(2), DriveConstants.TrainConstants.kDriveMotorGearRatio,
			.85, DriveConstants.TrainConstants.weight,
			DriveConstants.TrainConstants.kWheelDiameter / 2,
			DriveConstants.kChassisWidth, null);
	private double leftAppliedVolts = 0.0;
	private double rightAppliedVolts = 0.0;
	private boolean closedLoop = false;
	private PIDController leftPID = new PIDController(KP, 0.0, KD);
	private PIDController rightPID = new PIDController(KP, 0.0, KD);
	private double leftFFVolts = 0.0;
	private double rightFFVolts = 0.0;

	@Override
	public void updateInputs(TankIOInputs inputs) {
		if (closedLoop) {
			leftAppliedVolts = MathUtil.clamp(leftPID.calculate(
					sim.getLeftVelocityMetersPerSecond() / Tank.WHEEL_RADIUS)
					+ leftFFVolts, -12.0, 12.0);
			rightAppliedVolts = MathUtil.clamp(rightPID.calculate(
					sim.getRightVelocityMetersPerSecond() / Tank.WHEEL_RADIUS)
					+ rightFFVolts, -12.0, 12.0);
			sim.setInputs(leftAppliedVolts, rightAppliedVolts);
		}
		sim.update(0.02);
		inputs.leftPositionRad = sim.getLeftPositionMeters() / Tank.WHEEL_RADIUS;
		inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond()
				/ Tank.WHEEL_RADIUS;
		inputs.leftAppliedVolts = leftAppliedVolts;
		inputs.leftCurrentAmps = new double[] { sim.getLeftCurrentDrawAmps()
		};
		inputs.rightPositionRad = sim.getRightPositionMeters()
				/ Tank.WHEEL_RADIUS;
		inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond()
				/ Tank.WHEEL_RADIUS;
		inputs.rightAppliedVolts = rightAppliedVolts;
		inputs.rightCurrentAmps = new double[] { sim.getRightCurrentDrawAmps()
		};
		inputs.gyroYaw = sim.getHeading();
	}

	@Override
	public void setVoltage(double leftVolts, double rightVolts) {
		closedLoop = false;
		leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
		rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
		sim.setInputs(leftAppliedVolts, rightAppliedVolts);
	}

	@Override
	public void setVelocity(double leftRadPerSec, double rightRadPerSec,
			double leftFFVolts, double rightFFVolts) {
		closedLoop = true;
		leftPID.setSetpoint(leftRadPerSec);
		rightPID.setSetpoint(rightRadPerSec);
		this.leftFFVolts = leftFFVolts;
		this.rightFFVolts = rightFFVolts;
	}
}
