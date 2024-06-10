package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.drive.DriveConstants;
import frc.robot.utils.drive.PathFinder;

public class BranchSecondV extends Command{
	private boolean isFinished = false;
	private 	Command path;
	public BranchSecondV( ){
	}
	public void initialize(){
			isFinished = false;
			if (RobotContainer.currentNote == 0){
				path = AutoBuilder.followPath(PathPlannerPath.fromChoreoTrajectory("Second V"));
			}else{
				path = PathFinder.goToPose(new Pose2d(7.3,4.11,new Rotation2d()),DriveConstants.pathConstraints, RobotContainer.drivetrainS,true);
			}
			path.initialize();
	}
	@Override
	public void execute() {
		 if (path != null) {
			path.execute();
			  if (path.isFinished()) {
				path.end(false);
					isFinished = true;
			  }
		 }
	}
	@Override
	public void end(boolean interrupted) {
		 if (path != null) {
			  path.end(interrupted);
		 }
	}

	@Override
	public boolean isFinished(){
		return isFinished;
	}
}
