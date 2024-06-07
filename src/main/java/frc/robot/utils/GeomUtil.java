package frc.robot.utils;

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
}
