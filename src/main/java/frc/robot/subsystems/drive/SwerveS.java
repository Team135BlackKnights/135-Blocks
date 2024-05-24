package frc.robot.subsystems.drive;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public interface SwerveS extends Subsystem{
	void setChassisSpeeds(ChassisSpeeds speeds);
   ChassisSpeeds getChassisSpeeds();
   void resetPose(Pose2d pose);
   void newVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> estStdDevs);
   Pose2d getPose();
	void stopModules();
	Command sysIdDynamicTurn(Direction kreverse);
	Command sysIdQuasistaticTurn(Direction kreverse);
	Command sysIdDynamicDrive(Direction kforward);
	Command sysIdQuasistaticDrive(Direction kreverse);
	void zeroHeading();
} 
