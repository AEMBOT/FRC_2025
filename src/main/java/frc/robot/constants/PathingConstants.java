package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

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

  public static final PIDConstants translationPIDConstants = new PIDConstants(5.0);
  public static final PIDConstants rotationPIDConstants = new PIDConstants(5.0);

  // TODO Constraints are placeholder. Figure out reasonable values.
  /** Constraints for the majority of driver-assist and auto paths. */
  public static final PathConstraints generalPathConstraints =
      new PathConstraints(
          1, 4, Radians.convertFrom(360, Degrees), Radians.convertFrom(360, Degrees));

  public static final class BasicTargets {}
}
