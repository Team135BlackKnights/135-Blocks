package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SubsystemChecker;
import frc.robot.utils.GeomUtil;
import frc.robot.utils.vision.VisionConstants;
import frc.robot.utils.vision.VisionConstants.PVCameras;

public class Vision extends SubsystemChecker{
	private final VisionIO io;
	private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
	private boolean override = false;
	private Timer timer = new Timer();
	private PhotonCamera[] cams;
	public final PhotonPoseEstimator[] camEstimates;
	public Vision(VisionIO io){
		this.io = io;
		this.cams = io.getCams();
		this.camEstimates = io.getEstimators();
	}
	@Override
	public void periodic(){
		io.updateInputs(inputs);
		Logger.processInputs("VisionS", inputs);
		if (timer.get() > 1){
			timer.stop();
			override = false;
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
					
					double time = est.timestampSeconds*1.001e6;
					String response = shouldAcceptVision(time, estPose,
							RobotContainer.drivetrainS.getPose(),
							RobotContainer.drivetrainS.getChassisSpeeds(),
							aprilTagList,cCam);
					if (response == "OK") {
						// Change our trust in the measurement based on the tags we can see
						// We do this because we should trust cam estimates with closer apriltags than farther ones.
						Matrix<N3, N1> estStdDevs = getEstimationStdDevs(estPose,
								cEstimator, cCam);
						System.out.println("GOIN");
						addVisionMeasurement(est.estimatedPose.toPose2d(),
								time, estStdDevs);
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
				//addFault(cCam.getName() + "CAMERA IS NOT DETECTED", false, true);
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
			ArrayList<Integer> aprilTagList, PhotonCamera cam) {
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
		boolean[] moduleSkids = RobotContainer.drivetrainS.isSkidding();
		if (moduleSkids[0] || moduleSkids[1] || moduleSkids[2] || moduleSkids[3] || RobotContainer.drivetrainS.isCollisionDetected() || override){
			if (RobotContainer.drivetrainS.isCollisionDetected()){
				override = true;
				timer.restart();
			}
			if (GeomUtil.distancePose(lastPosition,
			newEst) > VisionConstants.kMaxVisionCorrectionSkid){
				SmartDashboard.putString("Vision validation", "Max correction");
				return "Max correction";
			}
			if (Math
				.abs(newEst.getRotation().minus(lastPosition.getRotation())
						.getRadians()) > VisionConstants.kMaxRotationCorrectionSkid
				&& (RobotContainer.drivetrainS.isConnected())) {
					SmartDashboard.putString("Vision validation", "Max rotation");
					return "Max rotation";
				}
			var targets = cam.getLatestResult().getTargets();
			for (var target : targets){
				if (target.getPoseAmbiguity() > .5){
					SmartDashboard.putString("Vision validation", "Max ambiguity");
					return "Max ambiguity";
				}
			}
			SmartDashboard.putString("Vision validation", "Override");
			return "OK";
		}
		//The following check for velocity MAY be unneeded!
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
		var targets = cam.getLatestResult().getTargets();
		for (var target : targets){
			if (target.getPoseAmbiguity() > .3){
				SmartDashboard.putString("Vision validation", "Max ambiguity");
				return "Max ambiguity";
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
		//All the targets that have been pulled from a camera
		var targets = cam.getLatestResult().getTargets();
		//Number of tags and average distance
		numTags.clear();
		double avgDist = 0, avgPoseAmbiguity = 0;
		double lowestDist = Double.MAX_VALUE;
		//Iterate through each target
		for (var tgt : targets) {
			//Return the distance from the individual tag and the number of tags
			numTags.add(tgt.getFiducialId());
			var tagPose = photonEstimator.getFieldTags()
					.getTagPose(tgt.getFiducialId());
			if (tagPose.isEmpty())
				continue;
			double dist = tagPose.get().toPose2d().getTranslation()
					.getDistance(estimatedPose.getTranslation());
			if (dist < lowestDist) {
				lowestDist = dist;
			}
			avgDist += dist;
			avgPoseAmbiguity += tgt.getPoseAmbiguity();
		}
		avgDist /= numTags.size();
		avgPoseAmbiguity /= numTags.size();
		double weighAverage = 0;
		for (int tag : numTags) {
			weighAverage += VisionConstants.FieldConstants.aprilTagOffsets[tag];
		}
		weighAverage /= numTags.size();
		double xyStdDev = calculateXYStdDev(lowestDist, avgDist, avgPoseAmbiguity,
				weighAverage, numTags.size());
		double thetaStdDev = calculateThetaStdDev(lowestDist, avgDist,
				avgPoseAmbiguity, weighAverage, numTags.size());
		xyStdDev = applyAdditionalAdjustments(xyStdDev,numTags.size());
		thetaStdDev = applyAdditionalAdjustments(thetaStdDev,numTags.size());
		return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
	}

	/**
	 * Calculate xyStdDev based on distance, average distance, average pose
	 * ambiguity, weigh average, and number of tags.
	 */
	private double calculateXYStdDev(double lowestDist, double avgDist,
			double avgPoseAmbiguity, double weighAverage, int numTags) {
		double distWeight = Math.pow(lowestDist / 2.0, 2.0);
		double avgDistWeight = Math.pow(avgDist / 3.5,2.0);
		double poseWeight = Math.pow(avgPoseAmbiguity / 0.2, 2.0);
		double weighAverageWeight = Math.pow(weighAverage, 2.0);
		return VisionConstants.std_dev_multiplier * (distWeight + poseWeight + weighAverageWeight + avgDistWeight) / (numTags * 2);
	}

	/**
	 * Calculate thetaStdDev based on distance, average distance, average pose
	 * ambiguity, weigh average, and number of tags.
	 */
	private double calculateThetaStdDev(double lowestDist, double avgDist,
			double avgPoseAmbiguity, double weighAverage, int numTags) {
		double distWeight = Math.pow(lowestDist / 2.0, 2.0);
		double avgDistWeight = Math.pow(avgDist / 3.5,2.0);
		double poseWeight = Math.pow(avgPoseAmbiguity / 0.2, 2.0);
		double weighAverageWeight = Math.pow(weighAverage, 2.0);

    	return VisionConstants.std_dev_multiplier * (distWeight + poseWeight + weighAverageWeight + avgDistWeight) / (numTags * 2);
	}

	/**
	 * Apply any additional adjustments or constraints to the calculated standard
	 * deviation.
	 */
	private double applyAdditionalAdjustments(double stdDev, double tagCount) {
		boolean[] moduleSkids = RobotContainer.drivetrainS.isSkidding();
		for (boolean moduleSkid : moduleSkids){
			if (moduleSkid){
				stdDev /= 2;
			}
		}
		if (RobotContainer.drivetrainS.isCollisionDetected() || override){
			stdDev /= 4;
		}
		if (moduleSkids[0] || moduleSkids[1] || moduleSkids[2] || moduleSkids[3]){
			return Math.max(0.05,stdDev);
		}
		if (tagCount == 1){
			return Math.max(0.35,stdDev);
		}else if (tagCount == 2){
			return Math.max(0.2,stdDev);
		}
		// Add more adjustments as needed
		return Math.max(0.02, stdDev); // Minimum threshold
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
			visionEst.ifPresentOrElse(est -> io.updateVisionSim(objectName,est.estimatedPose.toPose2d()),() -> {
				if (newResult)
					io.updateVisionObject(objectName);
			});
		}
	}
	
	@Override
	public List<ParentDevice> getOrchestraDevices() {
		return Collections.emptyList();
	}

	@Override
	protected Command systemCheckCommand() { return Commands.none(); }

	@Override
	public double getCurrent() { return 0; }
	
}
