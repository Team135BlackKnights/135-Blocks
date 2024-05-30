package frc.robot.commands.state_space;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.state_space.ElevatorS;
import frc.robot.utils.state_space.StateSpaceConstants;

public class ElevatorC extends Command {
	private final ElevatorS elevatorS;

	public ElevatorC(ElevatorS elevatorS) {
		this.elevatorS = elevatorS;
		addRequirements(elevatorS);
	}

	@Override
	public void initialize() {
		//do nothing, armS handles startup.
	}

	@Override
	public void execute() {
		double elevatorSpeed = 0, elevatorPos = ElevatorS.getSetpoint();
		double desSpeed = -RobotContainer.manipController.getLeftY();
		if (StateSpaceConstants.Controls.go2ftButton.getAsBoolean()) {
			elevatorPos = Units.feetToMeters(2);
		} else if (StateSpaceConstants.Controls.go0ftButton.getAsBoolean()) {
			elevatorPos = Units.feetToMeters(0);
		}
		if (Math.abs(desSpeed) > StateSpaceConstants.Controls.kArmDeadband) {
			if (elevatorPos >= StateSpaceConstants.Elevator.maxPosition && desSpeed > 0) {
				elevatorPos = StateSpaceConstants.Elevator.maxPosition;
			} else if (elevatorPos <= StateSpaceConstants.Elevator.startingPosition
					&& desSpeed < 0) {
				elevatorPos = StateSpaceConstants.Elevator.startingPosition;
			} else {
				elevatorSpeed = desSpeed * StateSpaceConstants.Elevator.maxAcceleration
						* StateSpaceConstants.Controls.elevatorMoveSpeed;
				elevatorPos += (elevatorSpeed * ElevatorS.dtSeconds); //add to our current position 20 MS of that accel
			}
		}
		if (elevatorSpeed == 0) {
			elevatorS.moveElevator(elevatorS.createState(elevatorPos));
		} else {
			elevatorS.moveElevator(elevatorS.createState(elevatorPos, elevatorSpeed));
		}
	}

	@Override
	public void end(boolean interrupted) {}

	@Override
	public boolean isFinished() { return false; }
}
