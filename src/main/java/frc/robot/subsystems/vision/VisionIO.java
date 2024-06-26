package frc.robot.subsystems.vision;


import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;


public interface VisionIO {
	@AutoLog
	public static class VisionIOInputs {
		
	}
	/** Updates the set of loggable inputs. */
	public default void updateInputs(VisionIOInputs inputs) {}
	public default void updateVisionSim(String objectName, Pose2d estimatedPose){}
	public default void updateVisionObject(String objectName){}
	public default PhotonCamera[] getCams(){return null;}
	public default PhotonPoseEstimator[] getEstimators(){return null;}

}
