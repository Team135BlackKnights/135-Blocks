package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeomUtil {
	  /**
   * Creates a pure translating transform
   *
   * @param x The x componenet of the translation
   * @param y The y componenet of the translation
   * @return The resulting transform
   */
  public static Transform2d translationToTransform(double x, double y) {
    return new Transform2d(new Translation2d(x, y), new Rotation2d());
  }
  public static Rotation2d rotationFromCurrentToTarget(Pose2d currentPose, Pose2d targetPose) {
        // Extract positions
        double dx = targetPose.getX() - currentPose.getX();
        double dy = targetPose.getY() - currentPose.getY();

        // Compute angle from currentPose to targetPose
        double angle = Math.atan2(dy, dx);

        // Convert angle from radians to degrees
        double angleDegrees = Math.toDegrees(angle);

        // Create a Rotation2d object from the angle
        Rotation2d rotationFromCurrentToTarget = new Rotation2d(angleDegrees);

        return rotationFromCurrentToTarget;
    }
}
