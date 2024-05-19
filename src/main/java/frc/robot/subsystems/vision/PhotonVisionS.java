package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.PVCameras;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.drive.SwerveS;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionS extends SubsystemBase{
	public static VisionSystemSim visionSim;
	//These estimate pose based on april tag location
	public final PhotonPoseEstimator rightEstimator,
			//frontEstimator,
			//leftEstimator,
			backEstimator;
	//Put these values into an array to iterate through them(go through them in order)
	public final PhotonPoseEstimator[] camEstimates;
	public final PhotonCamera[] cams;
	//Holds the timestamp at which the last pose was taken
	public double frontLastEstTimestamp = 0f, rightLastEstTimestamp = 0f,
			leftLastEstTimestamp = 0f, backLastEstTimestamp = 0f;
	//Used for aligning with a particular aprilTag
	public static double backCamXError = 0f;
	static double distance = 0;
	@SuppressWarnings("unused")
	private static PhotonCamera frontCam, rightCam, leftCam, backCam;

	public PhotonVisionS() {
		//load the field 
		AprilTagFieldLayout fieldLayout = AprilTagFieldLayout
				.loadField(AprilTagFields.k2024Crescendo);
		//Create the cameras
		rightCam = new PhotonCamera(VisionConstants.rightCamName);
		backCam = new PhotonCamera(VisionConstants.backCamName);
		//frontCam = new PhotonCamera(VisionConstants.frontCamName);
		//leftCam = new PhotonCamera(VisionConstants.leftCamName);
		//Set the pipelines (how it computes what we want), similar to limelight 
		backCam.setPipelineIndex(0);
		rightCam.setPipelineIndex(0);
		//frontCam.setPipelineIndex(0);
		//leftCam.setPipelineIndex(0);
		//Create new photon pose estimators
		rightEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightCam,
				VisionConstants.robotToRight);
		backEstimator = new PhotonPoseEstimator(VisionConstants.kTagLayout,
				PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCam,
				VisionConstants.robotToBack);
		/*leftEstimator = new PhotonPoseEstimator(
			VisionConstants.kTagLayout, 
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
			leftCam, 
			VisionConstants.robotToLeft);*/
		/*frontEstimator = new PhotonPoseEstimator(
			VisionConstants.kTagLayout, 
			PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
			frontCam,VisionConstants.robotToFront);*/
		//Set the strategy to use if for some reason multitags don't work
		rightEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		backEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		/*frontEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);*/
		/*leftEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);*/
		//Puts the estimators in an array to iterate through them efficiently
		camEstimates = new PhotonPoseEstimator[] { rightEstimator,
				backEstimator/*,
					frontEstimator,
					leftEstimator */
		};
		//Same process for cameras
		cams = new PhotonCamera[] { 
			rightCam, 
			backCam/*,
			leftCam,
			frontCam */
		};
		//Create PhotonVision camera simulations if the robot is in sim
		if (Constants.currentMode == Mode.SIM) {
			//Array for iteration
			PhotonCameraSim[] cameraSims = new PhotonCameraSim[] {
					new PhotonCameraSim(rightCam), new PhotonCameraSim(backCam),
					//new PhotonCameraSim(leftCam),
					//new PhotonCameraSim(rightCam)
			};
			//Handles the vision simulation
			visionSim = new VisionSystemSim("Vision Sim");
			//Adds the april tags
			visionSim.addAprilTags(fieldLayout);
			/*We use ARDUCAM OV9281s, which are the same cameras as on the limelight. 
			This means we can use the limelight properties. In an array for iteration.*/
			SimCameraProperties[] properties = new SimCameraProperties[] {
					SimCameraProperties.LL2_960_720(),
					SimCameraProperties.LL2_960_720()/*,
					SimCameraProperties.LL2_960_720(),
					SimCameraProperties.LL2_960_720(),
																*/
			};
			//Adds each camera present in the previous array to the visionSim. 
			for (var i = 0; i < cams.length; i++) {
				cameraSims[i] = new PhotonCameraSim(cams[i], properties[i]);
				visionSim.addCamera(cameraSims[i],
						VisionConstants.camTranslations[i]);
				cameraSims[i].enableRawStream(true);
				cameraSims[i].enableProcessedStream(true);
				cameraSims[i].enableDrawWireframe(true);
			}
		}
	}
	
	
	/**
	 * Computes the distance in inches from the limelight network table entry
	 * 
	 * @param tY the tY measurement from the limelight network table entry
	 * @return distance in inches
	 */
	public static double calculateDistanceFromtY(double tY) {
		// Calculate the angle using trigonometry
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = VisionConstants.limeLightAngleOffsetDegrees;
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = VisionConstants.limelightLensHeightoffFloorInches;
		// distance from the target to the floor
		double goalHeightInches = 0; //try zero? try two?
		double angleToGoalDegrees = limelightMountAngleDegrees + tY;
		//calculate distance
		double distance = (goalHeightInches - limelightLensHeightInches)
				/ Math.tan(Units.degreesToRadians(angleToGoalDegrees));
		return Math.abs(distance); //incase somehow the angle became pos when the object is below the target, and vice versa.
	}

	/**
	 * Uses one particular camera to figure out if the AprilTags in the argument
	 * is within sight or not
	 * 
	 * @param Camera        the camera to use
	 * @param tagsToLookFor the potential aprilTags to look for
	 * @return Whether a requested april tag is visible (as a boolean)
	 */
	public static boolean aprilTagVisible(PhotonCamera Camera,
			int[] tagsToLookFor) {
		var results = Camera.getLatestResult().getTargets();
		for (int targetTagNumber : tagsToLookFor) {
			for (var target : results) {
				if (target.getFiducialId() == targetTagNumber) {
					return true;
				}
			}
		}
		return false;
	}

	/**
	 * If the robot is in simulation, updates the states of the simulated camera
	 * and the simulated robot pose
	 * 
	 * @param visionEst The output from the getPhotonPoseEstimate()
	 * @param newResult Checks to see whether to set them to the field
	 * @param camera    The camera to use, as a custom PVCam enum declared in
	 *                     visionConstants
	 */
	private void updateSimField(Optional<EstimatedRobotPose> visionEst,
			boolean newResult, VisionConstants.PVCameras camera) {
		if (Constants.currentMode == Mode.SIM) {
			//Names the object based on the camera
			String objectName;
			switch (camera) {
			case Front_Camera:
				objectName = "VisionEstimationF";
				break;
			case Right_Camera:
				objectName = "VisionEstimationR";
				break;
			case Left_Camera:
				objectName = "VisionEstimationL";
				break;
			default:
				objectName = "VisionEstimationB";
				break;
			}
			//Sets the position of the robot on the simulation field if there are new results
			visionEst.ifPresentOrElse(est -> visionSim.getDebugField()
					.getObject(objectName).setPose(est.estimatedPose.toPose2d()),
					() -> {
						if (newResult)
							visionSim.getDebugField().getObject(objectName).setPoses();
					});
		}
	}

	/**
	 * Estimates the simulator global pose based on an individual photon
	 * estimator and updates the sim field if its in simulation
	 * 
	 * @param photonEstimator the estimator to use
	 * @param camera          the camera to use
	 * @param prevEstPose2d   the previous pose of the estimator
	 * @return the pose estimator (if it exists)
	 */
	public Optional<EstimatedRobotPose> getEstimatedGlobalPose(
			PhotonPoseEstimator photonEstimator, PhotonCamera camera,
			Pose2d prevEstPose2d) {
		//Feed it previous pose
		photonEstimator.setReferencePose(prevEstPose2d);
		//Update the estimated position
		var visionEst = photonEstimator.update();
		//Return the timestamp, check results
		double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
		boolean newResult = false;
		//Figure out what camera it is
		PVCameras cameraVal;

		switch (camera.getName()) {
		case VisionConstants.frontCamName:
			newResult = Math.abs(latestTimestamp - frontLastEstTimestamp) > 1e-5;
			if (newResult)
				frontLastEstTimestamp = latestTimestamp;
			cameraVal = PVCameras.Front_Camera;
			break;
		case VisionConstants.leftCamName:
			newResult = Math.abs(latestTimestamp - leftLastEstTimestamp) > 1e-5;
			if (newResult)
				leftLastEstTimestamp = latestTimestamp;
			cameraVal = PVCameras.Left_Camera;
			break;
		case VisionConstants.rightCamName:
			newResult = Math.abs(latestTimestamp - rightLastEstTimestamp) > 1e-5;
			if (newResult)
				rightLastEstTimestamp = latestTimestamp;
			cameraVal = PVCameras.Right_Camera;
			break;
		//By elimination it means that this becomes back camera
		default:
			cameraVal = PVCameras.Back_Camera;
			if (newResult)
				backLastEstTimestamp = latestTimestamp;
			break;
		}
		if (Robot.isSimulation()) {
			updateSimField(visionEst, newResult, cameraVal);
		}
		return visionEst;
	}
	/**Returns the field as seen by photonVision */
	public static Field2d getEstimatedField() {
		return visionSim.getDebugField();
	}

}

