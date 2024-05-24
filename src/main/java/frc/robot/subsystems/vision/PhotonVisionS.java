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
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.subsystems.drive.REVSwerve.SwerveModules.REVSwerveS;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionS extends SubsystemBase {
	public static VisionSystemSim visionSim;
	//These estimate pose based on april tag location
	public final PhotonPoseEstimator rightEstimator, frontEstimator,
			leftEstimator, backEstimator;
	//Put these values into an array to iterate through them(go through them in order)
	public final PhotonPoseEstimator[] camEstimates;
	public final PhotonCamera[] cams;
	//Holds the timestamp at which the last pose was taken
	public double frontLastEstTimestamp = 0f, rightLastEstTimestamp = 0f,
			leftLastEstTimestamp = 0f, backLastEstTimestamp = 0f;
	//Used for aligning with a particular aprilTag
	public static double camXError = 0f;
	static double distance = 0;
	private static PhotonCamera frontCam, rightCam, leftCam, backCam;

	public PhotonVisionS() {
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
		rightEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		backEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		frontEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		leftEstimator.setMultiTagFallbackStrategy(
				PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
		//Puts the estimators in an array to iterate through them efficiently
		camEstimates = new PhotonPoseEstimator[] { rightEstimator, backEstimator,
				frontEstimator, leftEstimator
		};
		//Same process for cameras
		cams = new PhotonCamera[] { rightCam, backCam, leftCam, frontCam
		};
		//Create PhotonVision camera simulations if the robot is in sim
		if (Constants.currentMode == Mode.SIM) {
			//Array for iteration
			PhotonCameraSim[] cameraSims = new PhotonCameraSim[] {
					new PhotonCameraSim(rightCam), new PhotonCameraSim(backCam),
					new PhotonCameraSim(leftCam), new PhotonCameraSim(rightCam)
			};
			//Handles the vision simulation
			visionSim = new VisionSystemSim("Vision Sim");
			//Adds the april tags
			visionSim.addAprilTags(fieldLayout);
			/*We use ARDUCAM OV9281s, which are the same cameras as on the limelight 3G. BLACK AND WHITE 
			This means we can use the limelight properties. In an array for iteration.*/
			SimCameraProperties[] properties = new SimCameraProperties[] {
					SimCameraProperties.LL2_960_720(),
					SimCameraProperties.LL2_960_720(),
					SimCameraProperties.LL2_960_720(),
					SimCameraProperties.LL2_960_720(),
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

	/**
	 * Computes the distance in inches from the limelight network table entry
	 * 
	 * @param tY the tY measurement from the limelight network table entry (pitch
	 *              degrees)
	 * @return distance in inches
	 */
	public static double calculateDistanceFromtY(double tY) {
		// Calculate the angle using trigonometry
		// how many degrees back is your limelight rotated from perfectly vertical?
		double limelightMountAngleDegrees = VisionConstants.limeLightAngleOffsetDegrees;
		// distance from the center of the Limelight lens to the floor
		double limelightLensHeightInches = VisionConstants.limelightLensHeightoffFloorInches;
		// distance from the target to the floor
		double goalHeightInches = 0; //if multiple targets, make this an argument
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
	public static boolean aprilTagVisible(PhotonCamera Camera, int[] tagsToLookFor) {
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

	/** Returns the field as seen by photonVision */
	public static Field2d getEstimatedField() {
		return visionSim.getDebugField();
	}

	@Override
	public void periodic() {
		//If code's in sim update the simulated pose estimator
		if (Constants.currentMode == Mode.SIM) {
			visionSim.update(REVSwerveS.getPose());
		}
		//Update the global pose for each camera
		for (int i = 0; i < cams.length; i++) {
			PhotonPoseEstimator cEstimator = camEstimates[i];
			PhotonCamera cCam = cams[i];
			var visionEst = getEstimatedGlobalPose(cEstimator, cCam,
					REVSwerveS.getPose());
			visionEst.ifPresent(est -> {
				Pose2d estPose = est.estimatedPose.toPose2d();
				// Change our trust in the measurement based on the tags we can see
				// We do this because we should trust cam estimates with closer apriltags than farther ones.
				Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose,
						cEstimator, cCam);
				SmartDashboard.putString("CAMERAUPDATE", cCam.getName());
				addVisionMeasurement(est.estimatedPose.toPose2d(),
						est.timestampSeconds, estStdDevs);
			});
		}
	}

	/**
	 * Uses one particular camera to check the x distance (in degrees) of the
	 * robot to a particular AprilTag, and saves it in the double called
	 * camXError. Should be called in periodic. If you want to check for tags
	 * based on alliance, surround this in robot.isRed and adjust the
	 * parameters as such
	 * 
	 * @param camera Camera to use
	 * @param tagIDs the tag ID to look for
	 */
	public void checkAutoLock(PhotonCamera camera, int tagID) {
		//Get a list of all sighted targets
		var targets = camera.getLatestResult().getTargets();
		//By default it can't see the speaker to avoid unintended behavior
		boolean hasSpeaker = false;
		for (var target : targets) {
			/*If the tagID is there, set the xError to the horizontal distance in degrees*/
			if (target.getFiducialId() == tagID) {
				hasSpeaker = true;
				camXError = -target.getBestCameraToTarget().getY(); //Y is distance from CENTER, left Right. 
			}
		}
		//If we don't have the ID set the x Error to zero to make it obvious that we don't have access to it
		if (!hasSpeaker) {
			camXError = 0;
		}
	}

	/**
	 * Adds the vision measurement to the poseEstimator. This is the same as the
	 * default poseEstimator function, we just do it this way to fit our block
	 * template structure
	 */
	public void addVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		REVSwerveS.poseEstimator.addVisionMeasurement(pose, timestamp, estStdDevs);
	}

	/**
	 * Compute the standard deviation based on the parameters given.
	 * 
	 * @param estimatedPose   the estimated pose that is being returned
	 * @param photonEstimator the pose estimator to use
	 * @param cam             the camera to use
	 * @return A matrix of the standard deviations
	 */
	public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose,
			PhotonPoseEstimator photonEstimator, PhotonCamera cam) {
		//Default Std deviation
		var estStdDevs = VisionConstants.kSingleTagStdDevs;
		//All the targets that have been pulled from a camera
		var targets = cam.getLatestResult().getTargets();
		//Number of tags and average distance
		int numTags = 0;
		double avgDist = 0;
		//Iterate through each target
		for (var tgt : targets) {
			//Return the distance from the individual tag and the number of tags
			var tagPose = photonEstimator.getFieldTags()
					.getTagPose(tgt.getFiducialId());
			if (tagPose.isEmpty())
				continue;
			if (tgt.getPoseAmbiguity() > .2)
				continue; //give zero F's about bad tags
			numTags++;
			avgDist += tagPose.get().toPose2d().getTranslation()
					.getDistance(estimatedPose.getTranslation());
		}
		//If no tags present, measurement is useless, so we make the deviations as big as we can so the poseEstimator ignores it
		if (numTags == 0)
			return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
					Double.MAX_VALUE); //BIG.
		//Compute the average distance 
		avgDist /= numTags;
		// Decrease std devs if multiple targets are visible
		if (numTags > 1)
			estStdDevs = VisionConstants.kMultiTagStdDevs;
		// Increase std devs based on (average) distance
		if (numTags == 1 && avgDist > 4)
			estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE,
					Double.MAX_VALUE);
		else
			//This is some stats nonsense. Essentially averaging out the distance over a given sensitivity of 30, then finding it on a normal curve.
			estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
		return estStdDevs;
	}

	/**
	 * Returns x error (in degrees). 
	 * @implSpec We used this as as an input to a PID loop to
	 * automatically face a speaker in 2024.
	 * @return the x error in degrees
	 */
	public static double getXError() {
		return PhotonVisionS.camXError;
	}

	/**
	 * Computes the distance of the robot from the specified target. If you want
	 * to use this in an alliance based way, surround it with
	 * Constants.isRed
	 * 
	 * @param targetLocation The target you want to get the distance from. Pose
	 *                          should be in meters and degrees.
	 * @return the distance from the robot to the speaker, in meters
	 */
	public static double getDistanceFromSpeakerUsingRobotPose(
			Pose2d targetLocation) {
		return REVSwerveS.robotPosition.getTranslation()
				.getDistance(targetLocation.getTranslation());
	}

	/**
	 * updates the xError double by using the robot's pose. This assumes that
	 * left is negative for tX. 
	 * IF ERROR IS OCCURING WITH TRACKING, THIS IS THE PROBLEM.
	 */
	public double updateXErrorWithPose(Pose2d targetPose) {
		return targetPose.getRotation().getDegrees()
				- REVSwerveS.getPose().getRotation().getDegrees();
	}

	/**
	 * Works similar to swerveS.optimize but for any angle
	 * 
	 * @param angle the angle to check
	 * @return the optimized distance to rotate to reach an ideal angle
	 */
	public static double closerAngleToZero(double angle) {
		// Normalize the angle to be within the range of -180 to 180 degrees
		double normalizedAngle = angle % 360;
		if (normalizedAngle > 180) {
			normalizedAngle -= 360;
		} else if (normalizedAngle < -180) {
			normalizedAngle += 360;
		}
		// Return the closer angle to zero
		return (Math.abs(normalizedAngle) <= 180) ? normalizedAngle
				: -normalizedAngle;
	}

	/**
	 * Modified PID Controller, but for our DriveToAITarget
	 * 
	 * @param x the x distance from target
	 * @return the x speed
	 */
	public static double speedMapper(double x) {
		// Define the parameters for the sigmoid function
		double x0 = 25; // Inches where the function starts to rise significantly
		double k = 0.1; // Steepness of the curve
		// Apply the sigmoid function to map x to the range [0, 1]
		double y = 1 / (1 + Math.exp(-k * (x - x0)));
		// Adjust the output to meet your specific points
		if (x >= 50) {
			y = 1;
		}
		return y;
	}
}
