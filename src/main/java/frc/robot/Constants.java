package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final DigitalInput robotJumper = new DigitalInput(0);
  public static final Robot currentRobot = Robot.DORY;
  public static final Mode currentMode =
      RobotBase.isReal()
          ? Mode.REAL
          : Mode.SIM; // You need to manually switch betweeen SIM and REPLAY.

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum Robot {
    DORY,
    NAUTILUS
  }

  public static final double UPDATE_PERIOD = 0.02;

  public static final class DriveConstants {
    // May need tweaking
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(18.5); // MK4i L3+
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(22.75); // 28 in square chassis
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(22.75);
    public static final double DRIVE_BASE_RADIUS =
        Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double CONTROLLER_DEADBAND = 0.05;
    public static final double SLOWMODE_MAX_METERS_PER_SEC = 1;
    public static final double SLOWMODE_ROTATION_SPEED_FACTOR = 0.2;

    public static final class Module {
      /* PORTS */
      public static final int TALON_DRIVE_MOTOR_0 = 7;
      public static final int TALON_TURN_MOTOR_0 = 8;
      public static final int TALON_CANCODER_0 = 26;

      public static final int TALON_DRIVE_MOTOR_1 = 5;
      public static final int TALON_TURN_MOTOR_1 = 6;
      public static final int TALON_CANCODER_1 = 24;

      public static final int TALON_DRIVE_MOTOR_2 = 3;
      public static final int TALON_TURN_MOTOR_2 = 4;
      public static final int TALON_CANCODER_2 = 25;

      public static final int TALON_DRIVE_MOTOR_3 = 9;
      public static final int TALON_TURN_MOTOR_3 = 2;
      public static final int TALON_CANCODER_3 = 23;

      public static final double WHEEL_RADIUS = Units.inchesToMeters(1.906);
      public static final double ODOMETRY_FREQUENCY = 200.0; // default 250, limited to 200 by NavX

      public static final Rotation2d[] absoluteEncoderOffset =
          switch (currentRobot) {
            case DORY ->
                new Rotation2d[] {
                  Rotation2d.fromRadians(2.291767297101148), // FL
                  Rotation2d.fromRadians(2.409883817768342 + Math.PI), // FR
                  Rotation2d.fromRadians(1.928213850372251), // BL
                  Rotation2d.fromRadians(1.73493227109866 + Math.PI) // BR
                };
            case NAUTILUS ->
                new Rotation2d[] { // This is not currently correct
                  Rotation2d.fromRadians(2.6599226861937018), // FL
                  Rotation2d.fromRadians(-2.9206994201342606 + Math.PI), // FR
                  Rotation2d.fromRadians(1.064582666792635), // BL
                  Rotation2d.fromRadians(-2.406815856192571 + Math.PI) // BR
                };
          };
    }
  }

  public static final class AprilTagConstants {
    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static enum CameraResolution {
      HIGH_RES,
      NORMAL
    }

    public final class NautilusCameras { // TODO Get actual Nautilus camera offsets
      public static final String frontLeftName = "front-left";
      public static final Transform3d frontLeftFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(11), Units.inchesToMeters(7), Units.inchesToMeters(4)),
              new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

      public static final String frontRightName = "front-right";
      public static final Transform3d frontRightFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(11), Units.inchesToMeters(-7), Units.inchesToMeters(4)),
              new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

      public static final String backLeftName = "back-left";
      public static final Transform3d backLeftFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-11), Units.inchesToMeters(11.5), Units.inchesToMeters(6)),
              new Rotation3d(
                  Units.degreesToRadians(180),
                  Units.degreesToRadians(-23.5),
                  Units.degreesToRadians(147)));

      public static final String backRightName = "back-right";
      public static final Transform3d backRightFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-11), Units.inchesToMeters(-11.5), Units.inchesToMeters(6)),
              new Rotation3d(
                  Units.degreesToRadians(180),
                  Units.degreesToRadians(-23.5),
                  Units.degreesToRadians(-147)));
    }

    public final class DoryCameras {
      public static final String frontLeftName = "front-left";
      public static final Transform3d frontLeftFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(11), Units.inchesToMeters(7), Units.inchesToMeters(4)),
              new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

      public static final String frontRightName = "front-right";
      public static final Transform3d frontRightFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(11), Units.inchesToMeters(-7), Units.inchesToMeters(4)),
              new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-30), 0.0));

      public static final String backLeftName = "back-left";
      public static final Transform3d backLeftFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-11), Units.inchesToMeters(11.5), Units.inchesToMeters(6)),
              new Rotation3d(
                  Units.degreesToRadians(180),
                  Units.degreesToRadians(-23.5),
                  Units.degreesToRadians(147)));

      public static final String backRightName = "back-right";
      public static final Transform3d backRightFromRobot =
          new Transform3d(
              new Translation3d(
                  Units.inchesToMeters(-11), Units.inchesToMeters(-11.5), Units.inchesToMeters(6)),
              new Rotation3d(
                  Units.degreesToRadians(180),
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
}
