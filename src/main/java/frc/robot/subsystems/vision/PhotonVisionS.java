package frc.robot.subsystems.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.ParentDevice;

import frc.robot.utils.GeomUtil;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.PVCameras;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.Mode;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import java.util.Collections;

public class PhotonVisionS extends SubsystemChecker {
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
	 * is within sight or not, then returning the Pose3d if found. ROBOT
	 * RELATIVE.
	 * 
	 * @param Camera       the camera to use
	 * @param tagToLookFor the potential aprilTag to look for
	 * @return An optional Pose3d which is ROBOT RELATIVE to the selected tag.
	 */
	public static Optional<Pose3d> aprilTagPose3d(PhotonCamera camera,
			int tagToLookFor) {
		var targets = camera.getLatestResult().getTargets();
		@SuppressWarnings("unlikely-arg-type")
		Optional<PhotonTrackedTarget> foundTargets = targets.stream()
				.filter(t -> t.getFiducialId() == tagToLookFor)
				.filter(t -> !t.equals(tagToLookFor) && t.getPoseAmbiguity() <= .8
						&& t.getPoseAmbiguity() != -1)
				.findFirst();
		if (foundTargets.isPresent()) {
			Transform3d cameraToTarget = foundTargets.get()
					.getBestCameraToTarget();
			return Optional.of(new Pose3d(cameraToTarget.getTranslation(),
					cameraToTarget.getRotation()));
		}
		return Optional.empty();
	}

	/**
	 * Create a field relative pose3d from a given pose3d
	 * 
	 * @param robotRelativePose the pose3d of the object RELATIVE to the robot
	 * @param robotPose         the robot pose
	 * @return field relative pose3d of the object
	 */
	public static Pose3d fieldRelativePose3d(Pose3d robotRelativePose,
			Pose2d robotPose) {
		return new Pose3d(robotPose)
				.transformBy(new Transform3d(robotRelativePose.getTranslation(),
						robotRelativePose.getRotation()));
	}

	public static boolean aprilTagVisible(PhotonCamera camera) {
		return !camera.getLatestResult().getTargets().isEmpty();
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
			case Left_Camera:
				objectName = "VisionEstimationL";
				break;
			case Right_Camera:
				objectName = "VisionEstimationR";
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
		boolean newResult = false;
		//Figure out what camera it is
		PVCameras cameraVal;
		switch (camera.getName()) {
		case VisionConstants.frontCamName:
			cameraVal = PVCameras.Front_Camera;
			break;
		case VisionConstants.leftCamName:
			cameraVal = PVCameras.Left_Camera;
			break;
		case VisionConstants.rightCamName:
			cameraVal = PVCameras.Right_Camera;
			break;
		//By elimination it means that this becomes back camera
		default:
			cameraVal = PVCameras.Back_Camera;
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
			visionSim.update(RobotContainer.drivetrainS.getPose());
		}
		//Update the global pose for each camera
		for (int i = 0; i < cams.length; i++) {
			PhotonPoseEstimator cEstimator = camEstimates[i];
			PhotonCamera cCam = cams[i];
			if (cCam.isConnected()) {
				var visionEst = getEstimatedGlobalPose(cEstimator, cCam,
						RobotContainer.drivetrainS.getPose());
				visionEst.ifPresent(est -> {
					Pose2d estPose = est.estimatedPose.toPose2d();
					ArrayList<Integer> aprilTagList = new ArrayList<>();
					for (PhotonTrackedTarget target : est.targetsUsed)
						aprilTagList.add(target.getFiducialId());
					/*for (PhotonTrackedTarget target : est.targetsUsed) { //To test the Offset feature, uncomment this.
						if (target.getFiducialId() == 6) {
							estPose = estPose.div(4012);
							//estPose = new Pose2d(0, 0, new Rotation2d(0,0));
						}
					}*/
					double time = est.timestampSeconds;
					String response = shouldAcceptVision(time, estPose,
							RobotContainer.drivetrainS.getPose(),
							RobotContainer.drivetrainS.getChassisSpeeds(),
							aprilTagList);
					if (response == "OK") {
						// Change our trust in the measurement based on the tags we can see
						// We do this because we should trust cam estimates with closer apriltags than farther ones.
						Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose,
								cEstimator, cCam);
						addVisionMeasurement(est.estimatedPose.toPose2d(),
								est.timestampSeconds, estStdDevs);
						for (int tag : numTags) {
							VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math
									.min(1,
											VisionConstants.FieldConstants.aprilTagOffsets[tag]
													+ .001);
						}
					} else {
						if (response == "Max correction") {
							for (int tag : numTags) {
								VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math
										.max(0,
												VisionConstants.FieldConstants.aprilTagOffsets[tag]
														- .001);
							}
						}
						if (response == "Min trust") {
							for (int tag : numTags) {
								VisionConstants.FieldConstants.aprilTagOffsets[tag] = Math
										.min(1,
												VisionConstants.FieldConstants.aprilTagOffsets[tag]
														+ .001);
							}
						}
					}
					if (VisionConstants.debug) {
						SmartDashboard.putString("CAMERAUPDATE", cCam.getName());
						SmartDashboard.putNumberArray("OFFSETS",
								VisionConstants.FieldConstants.aprilTagOffsets);
					}
				});
			} else {
				addFault(cCam.getName() + "CAMERA IS NOT DETECTED", false, true);
			}
		}
	}

	/**
	 * Adds the vision measurement to the poseEstimator. This is the same as the
	 * default poseEstimator function, we just do it this way to fit our block
	 * template structure
	 */
	public void addVisionMeasurement(Pose2d pose, double timestamp,
			Matrix<N3, N1> estStdDevs) {
		RobotContainer.drivetrainS.newVisionMeasurement(pose, timestamp,
				estStdDevs);
	}



	public String shouldAcceptVision(double timestamp, Pose2d newEst,
			Pose2d lastPosition, ChassisSpeeds robotVelocity,
			ArrayList<Integer> aprilTagList) {
		// Check out of field
		if (newEst.getTranslation()
				.getX() < -VisionConstants.FieldConstants.kFieldBorderMargin
				|| newEst.getTranslation()
						.getX() > VisionConstants.FieldConstants.kFieldLength
								+ VisionConstants.FieldConstants.kFieldBorderMargin
				|| newEst.getTranslation()
						.getY() < -VisionConstants.FieldConstants.kFieldBorderMargin
				|| newEst.getTranslation()
						.getY() > VisionConstants.FieldConstants.kFieldWidth
								+ VisionConstants.FieldConstants.kFieldBorderMargin) {
			SmartDashboard.putString("Vision validation", "Outside field");
			return "Outside field";
		}
		double overallChange;
		if (robotVelocity.vyMetersPerSecond == 0) {
			overallChange = Math.abs(robotVelocity.vxMetersPerSecond);
		} else {
			overallChange = Math.hypot(robotVelocity.vxMetersPerSecond,
					robotVelocity.vyMetersPerSecond);
		}
		if (overallChange > VisionConstants.kMaxVelocity) {
			SmartDashboard.putString("Vision validation", "Max velocity");
			return "Max velocity";
		}
		// Check max correction
		if (GeomUtil.distancePose(lastPosition,
				newEst) > VisionConstants.kMaxVisionCorrection) {
			SmartDashboard.putString("Vision validation", "Max correction");
			return "Max correction";
		}
		if (Math
				.abs(newEst.getRotation().minus(lastPosition.getRotation())
						.getRadians()) > VisionConstants.kMaxRotationCorrection
				&& (RobotContainer.drivetrainS.isConnected())) {
			SmartDashboard.putString("Vision validation", "Max rotation");
			return "Max rotation";
		}
		for (int aprilTagID : aprilTagList) {
			if (VisionConstants.FieldConstants.aprilTagOffsets[aprilTagID] < VisionConstants.FieldConstants.kFieldTagMinTrust) {
				SmartDashboard.putString("Vision validation", "Min trust");
			}
		}
		SmartDashboard.putString("Vision validation", "OK");
		return "OK";
	}

	/**
	 * Compute the standard deviation based on the parameters given.
	 * 
	 * @param estimatedPose   the estimated pose that is being returned
	 * @param photonEstimator the pose estimator to use
	 * @param cam             the camera to use
	 * @return A matrix of the standard deviations
	 */
	ArrayList<Integer> numTags = new ArrayList<>();

	public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose,
			PhotonPoseEstimator photonEstimator, PhotonCamera cam) {
		//Default Std deviation
		var estStdDevs = VisionConstants.kSingleTagStdDevs;
		//All the targets that have been pulled from a camera
		var targets = cam.getLatestResult().getTargets();
		//Number of tags and average distance
		numTags.clear();
		double avgDist = 0;
		double lowestDist = Double.MAX_VALUE;
		//Iterate through each target
		for (var tgt : targets) {
			//Return the distance from the individual tag and the number of tags
			numTags.add(tgt.getFiducialId());
			var tagPose = photonEstimator.getFieldTags()
					.getTagPose(tgt.getFiducialId());
			if (tagPose.isEmpty())
				continue;
			if (tgt.getPoseAmbiguity() > .2) {
				avgDist += 10;
				continue; //give zero F's about bad tags
			}
			double dist = tagPose.get().toPose2d().getTranslation()
					.getDistance(estimatedPose.getTranslation());
			if (dist < lowestDist) {
				lowestDist = dist;
			}
			avgDist += dist;
		}
		avgDist /= numTags.size();
		double xyStdDev = VisionConstants.std_dev_multiplier * (0.1)
				* ((0.01 * Math.pow(lowestDist, 2.0))
						+ (0.005 * Math.pow(avgDist, 2.0)))
				/ numTags.size();
		double weighAverage = 0;
		for (int tag : numTags) {
			weighAverage += VisionConstants.FieldConstants.aprilTagOffsets[tag];
		}
		weighAverage /= numTags.size();
		xyStdDev *= VisionConstants.std_dev_steepness
				* (1 - Math.exp(-16 * (1 - weighAverage))) + 1;
		xyStdDev = Math.max(0.01, xyStdDev);
		return estStdDevs;
	}



	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	@Override
	protected Command systemCheckCommand() { return Commands.none(); }
}
