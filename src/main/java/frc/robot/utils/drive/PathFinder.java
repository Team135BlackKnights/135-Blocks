package frc.robot.utils.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.subsystems.drive.DrivetrainS;
import frc.robot.RobotContainer;
public class PathFinder {
	/**
	 * Goes to a given pose with the speed constraints, and will ALWAYS end facing the given degree.
	 * @param pose desired end position
	 * @param constraints PathConstraints containing max speeds and velocities
	 * @param drive drivetrain type.
	 * @return pre-made command
	 */
	public static Command goToPose(Pose2d pose, PathConstraints constraints, DrivetrainS drive){
		return AutoBuilder.pathfindToPose((pose),constraints).andThen(new DriveToPose(drive,pose,constraints)).finallyDo(() -> 	RobotContainer.field.getObject("target pose").setPose(new Pose2d(-50,-50,new Rotation2d()))); //the void		));
	}
}
