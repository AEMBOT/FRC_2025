package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class VisionConstants {
  /**
   * The layout of the april tags on the field. Comps in PNW should use welded, and the differences
   * between welded and AndyMark are very small.
   */
  public static final AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  public static PoseStrategy poseStrategy = PoseStrategy.CONSTRAINED_SOLVEPNP;

  public static enum CameraResolution {
    HIGH_RES,
    NORMAL
  }

  public final class NautilusCameras {
    public static final String frontLeftName = "front-left";
    public static final Transform3d frontLeftFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9), Units.inchesToMeters(8.25), Units.inchesToMeters(7.5)),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

    public static final String frontRightName = "front-right";
    public static final Transform3d frontRightFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(9), Units.inchesToMeters(-8.25), Units.inchesToMeters(7.5)),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

    public static final String backLeftName = "back-left";
    public static final Transform3d backLeftFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.375), Units.inchesToMeters(8), Units.inchesToMeters(10.5)),
            new Rotation3d(
                Units.degreesToRadians(180),
                Units.degreesToRadians(-23.5),
                Units.degreesToRadians(147)));

    public static final String backRightName = "back-right";
    public static final Transform3d backRightFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-12.375),
                Units.inchesToMeters(-8),
                Units.inchesToMeters(10.5)),
            new Rotation3d(
                Units.degreesToRadians(180),
                Units.degreesToRadians(-23.5),
                Units.degreesToRadians(-147)));
  }

  public final class DoryCameras {
    /*
    TODO The coordinates of Dory's front camera offsets seem to be inverted along the Y axis
    compared to Dory's back and all of Nautilus' cameras.
    It'd be good to figure out why this happens. */
    public static final String frontLeftName = "front-left";
    public static final Transform3d frontLeftFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11), Units.inchesToMeters(-7), Units.inchesToMeters(4)),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-15), 0.0));

    public static final String frontRightName = "front-right";
    public static final Transform3d frontRightFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(11), Units.inchesToMeters(7), Units.inchesToMeters(4)),
            new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-15), 0.0));

    public static final String backLeftName = "back-left";
    public static final Transform3d backLeftFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11), Units.inchesToMeters(11.5), Units.inchesToMeters(6)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-23.5),
                Units.degreesToRadians(147)));

    public static final String backRightName = "back-right";
    public static final Transform3d backRightFromRobot =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-11), Units.inchesToMeters(-11.5), Units.inchesToMeters(6)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(-23.5),
                Units.degreesToRadians(-147)));
  }

  public static final Matrix<N3, N1> highResSingleTagStdDev =
      VecBuilder.fill(0.4, 0.4, Double.MAX_VALUE);
  public static final Matrix<N3, N1> normalSingleTagStdDev =
      VecBuilder.fill(0.8, 0.8, Double.MAX_VALUE);
  public static final Matrix<N3, N1> highResMultiTagStdDev = VecBuilder.fill(0.2, 0.2, 3);
  public static final Matrix<N3, N1> normalMultiTagStdDev =
      VecBuilder.fill(0.5, 0.5, Double.MAX_VALUE);
}
