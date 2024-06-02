package frc.robot.commands.state_space;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.FlywheelS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class FlywheelC extends Command {
	private final FlywheelS flywheelS;
	private double flywheelSpeed;

	public FlywheelC(FlywheelS flywheelS) {
		this.flywheelS = flywheelS;
		addRequirements(flywheelS);
	}

	@Override
	public void initialize() {
		//do not do anything to flywheels, flywheelS handles it.
	}

	@Override
	public void execute() {
		if (RobotContainer.manipController
				.getLeftTriggerAxis() > StateSpaceConstants.Controls.kDeadband) {
			flywheelSpeed = RobotContainer.manipController.getLeftTriggerAxis()
					* StateSpaceConstants.Flywheel.maxRPM;
		} else if (StateSpaceConstants.Controls.setButton.getAsBoolean()) {
			flywheelSpeed = 4000;
		} else {
			flywheelSpeed = 0;
		}
		flywheelS.setRPM(flywheelSpeed);
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
