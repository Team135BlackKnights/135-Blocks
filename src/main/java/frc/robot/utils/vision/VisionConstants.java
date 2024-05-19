package frc.robot.utils.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.drive.DriveConstants;

public class VisionConstants {
	//We used 2 cameras for our 2024 year, adjust these accordingly by removing camera names (there are two extra cameras here)
	//Camera names, from photonVision web interface
	public static String 
		frontCamName = "Front_Camera", backCamName = "Back_Camera", 
		leftCamName = "Left_Camera", rightCamName = "Right_Camera";
	public static String[] camNameStrings = new String[] 
	{ frontCamName, backCamName, 
	  leftCamName, rightCamName
	};
	//offset of each cam from robot center, in meters
	//To figure out what these should be, look at the WPILIB Coordinate System

	public static Translation3d 
				frontCamTranslation3d = new Translation3d(
						Units.inchesToMeters(15), 
						0, 
						Units.inchesToMeters(19.75)),
				rightCamTranslation3d = new Translation3d(
						Units.inchesToMeters(12.75),
						Units.inchesToMeters(
						DriveConstants.kChassisLength / 2),
						Units.inchesToMeters(19.75)),

				leftCamTranslation3d = new Translation3d(
						Units.inchesToMeters(12.75),
						Units.inchesToMeters(
						-DriveConstants.kChassisLength / 2),
						Units.inchesToMeters(19.75)),
				backCamTranslation3d = new Translation3d(
						Units.inchesToMeters(12.75), Units.inchesToMeters(0),
						Units.inchesToMeters(25));
		//Pitches of camera, in DEGREES, positive means UPWARD angle
		public static int
			//Figure these out on your robot
			frontCamPitch = -21, 
			rightCamPitch = -21, 
			leftCamPitch = -15, 
			backCamPitch = -10,

			//Camera resolution, and FPS
			camResWidth = 600,
			camResHeight = 800, 
			camFPS = 60;
		
		public static int[] camPitches = new int[] { 
			//Convert pitches into an array through iteration
			frontCamPitch, 
			rightCamPitch,
			leftCamPitch,
			backCamPitch
		};
		//Putting all the values together (Creating rotation3ds from the rotation 2ds and putting them together with the translation2ds)
		public static Translation3d 
		frontPos = frontCamTranslation3d,
		rightPos = rightCamTranslation3d,
		leftPos = leftCamTranslation3d,
		backPos = backCamTranslation3d;
		public static Rotation3d 
		frontRot = new Rotation3d(
			0,
			Math.toRadians(frontCamPitch),
			Math.toRadians(0)),
		rightRot = new Rotation3d(
			0,
			Math.toRadians(rightCamPitch),
			Math.toRadians(-90)),
		leftRot = new Rotation3d(
			0,
			Math.toRadians(leftCamPitch),
			Math.toRadians(90)),
		backRot = new Rotation3d(
			0,
			Math.toRadians(backCamPitch),
			Math.toRadians(180));

		//Transforms, used in the camera declarations
		@SuppressWarnings("unused")
		public static Transform3d 
		robotToFront = new Transform3d(frontPos, frontRot),
		robotToRight = new Transform3d(rightPos, rightRot),
		robotToLeft = new Transform3d(leftPos, leftRot),
		robotToBack = new Transform3d(backPos, backRot);

		//Put in an array for easier iterating
		Transform3d[] camTranslations = new Transform3d[] { robotToRight, robotToBack };

}
