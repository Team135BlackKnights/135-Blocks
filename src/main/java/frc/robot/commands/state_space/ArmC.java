package frc.robot.commands.state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.ArmS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ArmC extends Command {
	private final ArmS armS;
	private double armPos = 0;
	public ArmC(ArmS armS) {
		this.armS = armS;
		addRequirements(armS);
	}

	@Override
	public void initialize() {
		StateSpaceConstants.Controls.go45Button
						.onTrue(new InstantCommand(() -> {
							armPos = Units.degreesToRadians(45);
							armS.deployIntake(armS.createState(armPos));
						}));
		StateSpaceConstants.Controls.go0Button
						.onTrue(new InstantCommand(() -> {
							armPos = Units.degreesToRadians(0);
							armS.deployIntake(armS.createState(armPos));
						}));
	}

	@Override
	public void execute() {
		double armSpeed = 0, desSpeed = -RobotContainer.manipController.getRightY();
		armPos = ArmS.getSetpoint();
		if (StateSpaceConstants.Controls.go45Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(45);
		} else if (StateSpaceConstants.Controls.go0Button.getAsBoolean()) {
			armPos = Units.degreesToRadians(0);
		}
		if (Math.abs(desSpeed) > StateSpaceConstants.Controls.kArmDeadband) {
			if (armPos >= StateSpaceConstants.Arm.maxPosition && desSpeed > 0) {
				armPos = StateSpaceConstants.Arm.maxPosition;
			} else if (armPos <= StateSpaceConstants.Arm.startingPosition
					&& desSpeed < 0) {
				armPos = StateSpaceConstants.Arm.startingPosition;
			} else {
				armSpeed = desSpeed * StateSpaceConstants.Arm.maxAcceleration
						* StateSpaceConstants.Controls.armMoveSpeed;
				armPos += (armSpeed * ArmS.dtSeconds); //add to our current position 20 MS of that accel
			}
		}
		if (armSpeed == 0) {
			armS.deployIntake(armS.createState(armPos));
		} else {
			armS.deployIntake(armS.createState(armPos, armSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
