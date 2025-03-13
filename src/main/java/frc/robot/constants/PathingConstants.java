package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Rotation2d;

public class PathingConstants {
  public static final RobotConfig robotConfig;

  static {
    try {
      robotConfig =
          RobotConfig.fromGUISettings(); // TODO Set up this configuration in PathPlanner GUI
    } catch (Exception e) {
      throw new RuntimeException("Failed to initialize RobotConfig for PathPlanner", e);
    }
  }

  public static final PIDConstants translationPIDConstants = new PIDConstants(1.0);
  public static final PIDConstants rotationPIDConstants = new PIDConstants(5.0);

  /** The default xy tolerance to terminate a corrected path. */
  public static final double defaultTranslationTolerance = 0.01;

  /** The default theta tolerance to terminate a corrected path. */
  public static final Rotation2d defaultRotationTolerance = Rotation2d.fromDegrees(2.5);

  /**
   * If we try to path while too close to our target position, PathPlanner throws a fit. When
   * generating a corrected path, just use simple PID if the distance is under this amount.
   */
  public static final double minimumPathPlannerDistance = 0.5;

  // TODO Constraints are placeholder. Figure out reasonable values.
  /** Constraints for the majority of driver-assist and auto paths. */
  public static final PathConstraints generalPathConstraints =
      new PathConstraints(
          1, 4, Radians.convertFrom(360, Degrees), Radians.convertFrom(360, Degrees));

  public static final class BasicTargets {}
}
