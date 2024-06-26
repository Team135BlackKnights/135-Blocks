package frc.robot.subsystems.vision;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;


import frc.robot.utils.vision.VisionConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.Mode;

import frc.robot.RobotContainer;
import frc.robot.Constants;

public class VisionIOPhotonVision implements VisionIO{
	public static VisionSystemSim visionSim;
	//These estimate pose based on april tag location
	public final PhotonPoseEstimator rightEstimator, frontEstimator,
			leftEstimator, backEstimator;
	//Put these values into an array to iterate through them (go through them in order)
	public final PhotonPoseEstimator[] camEstimates;
	public final PhotonCamera[] cams;
	//Used for aligning with a particular aprilTag
	public static double camXError = 0f;
	static double distance = 0;
	private static PhotonCamera frontCam, rightCam, leftCam, backCam;
	public VisionIOPhotonVision() {
		//load the field 
		AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
				.loadField(AprilTagFields.k2024Crescendo);
		//Create the cameras
		rightCam = new PhotonCamera(VisionConstants.rightCamName);
		backCam = new PhotonCamera(VisionConstants.backCamName);
		frontCam = new PhotonCamera(VisionConstants.frontCamName);
		leftCam = new PhotonCamera(VisionConstants.leftCamName);
		//Set the pipelines (how it computes what we want), similar to limelight, default 0
		backCam.setPipelineIndex(0);
		rightCam.setPipelineIndex(0);
		frontCam.setPipelineIndex(0);
		leftCam.setPipelineIndex(0);
		//Create new photon pose estimators
		rightEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam,
				VisionConstants.robotToRight);
		backEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,
				VisionConstants.robotToBack);
		leftEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftCam,
				VisionConstants.robotToLeft);
		frontEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCam,
				VisionConstants.robotToFront);
		//Set the strategy to use when multitag isn't detected. IE, only one apriltag visible
		rightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		backEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		leftEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
		//Puts the estimators in an array to iterate through them efficiently
		camEstimates = new PhotonPoseEstimator[] { frontEstimator, leftEstimator,
				rightEstimator, backEstimator
		};
		//Same process for cameras
		cams = new PhotonCamera[] { frontCam, leftCam, rightCam, backCam
		};
		//Create PhotonVision camera simulations if the robot is in sim
		if (Constants.currentMode == Mode.SIM) {
			//Array for iteration
			PhotonCameraSim[] cameraSims = new PhotonCameraSim[] {
					new PhotonCameraSim(frontCam), new PhotonCameraSim(leftCam),
					new PhotonCameraSim(rightCam), new PhotonCameraSim(backCam)
			};
			//Handles the vision simulation
			visionSim = new VisionSystemSim("Vision Sim");
			//Adds the april tags
			visionSim.addAprilTags(fieldLayout);
			/*We use ARDUCAM OV9281s, which are the same cameras as on the limelight 3G. BLACK AND WHITE 
			This means we can use the limelight properties. In an array for iteration.*/
			SimCameraProperties[] properties = new SimCameraProperties[] {
					SimCameraProperties.LL2_1280_720(),
					SimCameraProperties.LL2_1280_720(),
					SimCameraProperties.LL2_1280_720(),
					SimCameraProperties.LL2_1280_720(),
			};
			//Adds each camera present in the previous array to the visionSim. 
			for (var i = 0; i < cams.length; i++) {
				cameraSims[i] = new PhotonCameraSim(cams[i], properties[i]);
				visionSim.addCamera(cameraSims[i],
						VisionConstants.camTranslations[i]);
				//the following are cool to see in sim, but take over 80ms to run, so use at risk for debugging
				cameraSims[i].enableRawStream(true);
				cameraSims[i].enableProcessedStream(true);
				cameraSims[i].enableDrawWireframe(true);
			}
		}
	}

	@Override
	public void updateVisionSim(String objectName, Pose2d estimatedPose){
		visionSim.getDebugField().getObject(objectName).setPose(estimatedPose);
	}
	@Override
	public void updateVisionObject(String objectName){
		visionSim.getDebugField().getObject(objectName).setPoses();
	}


	@Override
	public void updateInputs(VisionIOInputs inputs) {
		//If code's in sim update the simulated pose estimator
		if (Constants.currentMode == Mode.SIM) {
			visionSim.update(RobotContainer.drivetrainS.getPose());
		}
	}
	@Override
	public PhotonCamera[] getCams(){
		return cams;
	}
	@Override
	public PhotonPoseEstimator[] getEstimators(){
		return camEstimates;
	}
}
