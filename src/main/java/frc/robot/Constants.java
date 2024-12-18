package frc.robot;


import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.math.geometry.Rotation2d;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants{
  public static final DigitalInput robotJumper = new DigitalInput(0);
  public static final Robot CURRENT_ROBOT = robotJumper.get() ? Robot.SECONDARY_ROBOT : Robot.MAIN_ROBOT;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum Robot {
    MAIN_ROBOT,
    SECONDARY_ROBOT
  }

  public static final double UPDATE_PERIOD = 0.02;
  
  public static final class DriveConstants {
    // May need tweaking
    public static final double MAX_LINEAR_SPEED = Units.feetToMeters(0);
    public static final double TRACK_WIDTH_X = Units.inchesToMeters(0);
    public static final double TRACK_WIDTH_Y = Units.inchesToMeters(0);
    public static final double DRIVE_BASE_RADIUS =
      Math.hypot(TRACK_WIDTH_X / 2.0, TRACK_WIDTH_Y / 2.0);
    public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED / DRIVE_BASE_RADIUS;

    public static final double CONTROLLER_DEADBAND = 0.05;
    public static final double SLOWMODE_MAX_METERS_PER_SEC = 1;
    public static final double SLOWMODE_ROTATION_SPEED_FACTOR = 0.2;

    public static final class Module {
      /* PORTS */
      public static final int TALON_DRIVE_MOTOR_0 = 0;
      public static final int TALON_TURN_MOTOR_0 = 0;
      public static final int TALON_CANCODER_0 = 0;

      public static final int TALON_DRIVE_MOTOR_1 = 0;
      public static final int TALON_TURN_MOTOR_1 = 0;
      public static final int TALON_CANCODER_1 = 0;

      public static final int TALON_DRIVE_MOTOR_2 = 0;
      public static final int TALON_TURN_MOTOR_2 = 0;
      public static final int TALON_CANCODER_2 = 0;

      public static final int TALON_DRIVE_MOTOR_3 = 0;
      public static final int TALON_TURN_MOTOR_3 = 0;
      public static final int TALON_CANCODER_3 = 0;

      public static final double WHEEL_RADIUS = Units.inchesToMeters(0);
      public static final double ODOMETRY_FREQUENCY = 200.0; // default 250, limited to 200 by NavX

      public static final Rotation2d[] ABSOLUTE_ENCODER_OFFSETS = switch (CURRENT_ROBOT) {
        case MAIN_ROBOT -> new Rotation2d[] {
          Rotation2d.fromRadians(0), // FL
          Rotation2d.fromRadians(0), // FR
          Rotation2d.fromRadians(0), // BL
          Rotation2d.fromRadians(0) // BR
        };
        case SECONDARY_ROBOT -> new Rotation2d[] {
          Rotation2d.fromRadians(0), // FL
          Rotation2d.fromRadians(0), // FR
          Rotation2d.fromRadians(0), // BL
          Rotation2d.fromRadians(0) // BR
        };
      };
    }
  }

  /** Constants used primarily for the vision subsystem */
  public static final class VisionConstants {

    /** April tag field layout, inclues the positions of all static april tags as well as the size of the field. */ 
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT;
    static {
      AprilTagFieldLayout layout = null;
      try{
        layout = new AprilTagFieldLayout(Path.of(Filesystem.getDeployDirectory().toURI() + "\\aprilTagFieldLayout"));
      }
      catch (IOException e) {
        e.printStackTrace();
      }
      APRIL_TAG_FIELD_LAYOUT = layout;
  }
    

    // TODO determine these values  
    /** The name that connects the front camera in code to the front camera in photonvision on the rio */
    public static final String FRONT_CAM_NAME = "front";
    /** Transform from the center of the robot to the front camera */
    public static final Transform3d FRONT_CAM_FROM_ROBOT = switch (CURRENT_ROBOT) {
      case MAIN_ROBOT -> new Transform3d(
        new Translation3d(
            0,
            0,
            0),
        new Rotation3d(
            0, 
            0, 
            0)
      );
      case SECONDARY_ROBOT -> new Transform3d(
        new Translation3d(
            0,
            0,
            0),
        new Rotation3d(
            0, 
            0, 
            0)
        );
    };

    /** The name that connects the right camera in code to the right camera in photonvision on the rio */
    public static final String RIGHT_CAM_NAME = "right";
    /** Transform from the center of the robot to the right camera */
    public static final Transform3d RIGHT_CAM_FROM_ROBOT = switch (CURRENT_ROBOT) {
      case MAIN_ROBOT -> new Transform3d(
        new Translation3d(
            0,
            0,
            0),
        new Rotation3d(
            0, 
            0, 
            0)
      );
      case SECONDARY_ROBOT -> new Transform3d(
        new Translation3d(
            0,
            0,
            0),
        new Rotation3d(
            0, 
            0, 
            0)
        );
    };


    /** The name that connects the left camera in code to the left camera in photonvision on the rio */
    public static final String LEFT_CAM_NAME = "left";
    /** Transform from the center of the robot to the left camera */
    public static final Transform3d LEFT_CAM_FROM_ROBOT = switch (CURRENT_ROBOT) {
      case MAIN_ROBOT -> new Transform3d(
        new Translation3d(
            0,
            0,
            0),
        new Rotation3d(
            0, 
            0, 
            0)
      );
      case SECONDARY_ROBOT -> new Transform3d(
        new Translation3d(
            0,
            0,
            0),
        new Rotation3d(
            0, 
            0, 
            0)
        );
    };

    // Following Values are only used for sim

    /* *Vertical resolution of the front camera */
    public static final int FRONT_CAM_VERT_RES = 1000;
    /** Horizontal resolution of the front camera */
    public static final int FRONT_CAM_HORIZ_RES = 1000;
    /** Front camera rotation offset. */
    public static final Rotation2d FRONT_CAM_ROTATIONS = Rotation2d.fromDegrees(0);
    /** Front camera pixel error */
    public static final double FRONT_CAM_PIXEL_ERROR = 0;
    /** Standard Deviation in the pixel error of the front camera */
    public static final double FRONT_PIXEL_ERROR_STD_DEV = 0;
    /** How many frames the front camera provides per second */
    public static final double FRONT_CAM_FPS = 60;
    /** Average latency from the front camera */
    public static final double FRONT_CAM_AVG_LATENCY = 0;
    /** Standard deviation in the latency fro the front camera */
    public static final double FRONT_CAM_AVG_LATENCY_STD_DEV = 0;

    /* *Vertical resolution of the right camera */
    public static final int RIGHT_CAM_VERT_RES = 1000;
    /** Horizontal resolution of the right camera */
    public static final int RIGHT_CAM_HORIZ_RES = 1000;
    /** Right camera rotation offset. */
    public static final Rotation2d RIGHT_CAM_ROTATIONS = Rotation2d.fromDegrees(0);
    /** Right camera pixel error */
    public static final double RIGHT_CAM_PIXEL_ERROR = 0;
    /** Standard Deviation in the pixel error of the right camera */
    public static final double RIGHT_PIXEL_ERROR_STD_DEV = 0;
    /** How many frames the right camera provides per second */
    public static final double RIGHT_CAM_FPS = 60;
    /** Average latency from the right camera */
    public static final double RIGHT_CAM_AVG_LATENCY = 0;
    /** Standard deviation in the latency fro the right camera */
    public static final double RIGHT_CAM_AVG_LATENCY_STD_DEV = 0;

    /* *Vertical resolution of the left camera */
    public static final int LEFT_CAM_VERT_RES = 1000;
    /** Horizontal resolution of the left camera */
    public static final int LEFT_CAM_HORIZ_RES = 1000;
    /** Left camera rotation offset. */
    public static final Rotation2d LEFT_CAM_ROTATIONS = Rotation2d.fromDegrees(0);
    /** Left camera pixel error */
    public static final double LEFT_CAM_PIXEL_ERROR = 0;
    /** Standard Deviation in the pixel error of the left camera */
    public static final double LEFT_PIXEL_ERROR_STD_DEV = 0;
    /** How many frames the left camera provides per second */
    public static final double LEFT_CAM_FPS = 60;
    /** Average latency from the left camera */
    public static final double LEFT_CAM_AVG_LATENCY = 0;
    /** Standard deviation in the latency fro the left camera */
    public static final double LEFT_CAM_AVG_LATENCY_STD_DEV = 0;
  }
}