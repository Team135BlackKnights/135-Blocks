package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class RunTest extends Command {
	private final SysIdRoutine.Direction direction;
	private final boolean isQuasiastic;
	private Command test;

	public RunTest(SysIdRoutine.Direction direction, boolean isQuasiastic,
			Subsystem requirements) {
		this.direction = direction;
		this.isQuasiastic = isQuasiastic;
		addRequirements(requirements);
	}

	@Override
	public void initialize() {
		switch (Robot.runningTest) {

			case swerveDrive:
				if (isQuasiastic){
					test = RobotContainer.drivetrainS.sysIdQuasistaticDrive(direction);
				}else{
					test = RobotContainer.drivetrainS.sysIdDynamicDrive(direction);
				}
				break;
			default:
				System.err.println("NO GIVEN ROUTINE!");
				break;

		case elevator:
			if (isQuasiastic) {
				test = RobotContainer.elevatorS.sysIdQuasistatic(direction);
			} else {
				test = RobotContainer.elevatorS.sysIdDynamic(direction);
			}
			break;
		case flywheel:
			if (isQuasiastic) {
				test = RobotContainer.flywheelS.sysIdQuasistatic(direction);
			} else {
				test = RobotContainer.flywheelS.sysIdDynamic(direction);
			}
			break;
		case singleJointedArm:
			if (isQuasiastic) {
				test = RobotContainer.armS.sysIdQuasistatic(direction);
			} else {
				test = RobotContainer.armS.sysIdDynamic(direction);
			}
			break;
		default:
			System.err.println("NO GIVEN ROUTINE!");
			break;

		}
		test.schedule();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) { test.cancel(); }

	@Override
	public void cancel() { test.cancel(); }

	@Override
	public boolean isFinished() { return test.isFinished(); }
}
