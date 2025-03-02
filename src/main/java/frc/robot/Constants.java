package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public final class Constants {
  /**
   * A jumper that identifies what robot we're currently running on. It outputs high voltage on
   * Nautilus and low voltage on Dory. If there is no jumper, it'll default to high/true
   */
  public static final DigitalInput robotJumper = new DigitalInput(0);

  public static final Robot currentRobot;

  static {
    if (robotJumper.get()) {
      currentRobot = Robot.NAUTILUS;
    } else {
      currentRobot = Robot.DORY;
    }
  }

  public static final Mode currentMode =
      RobotBase.isReal()
          ? Mode.REAL
          : Mode.SIM; // You need to manually switch betweeen SIM and REPLAY.

  /** If true and in sim, use keyboard bindings instead of XBox controller. */
  public static final Boolean useKeyboard = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,
    /** Running a physics simulator. */
    SIM,
    /** Replaying from a log file. */
    REPLAY
  }

  public enum Robot {
    NAUTILUS,
    DORY
  }

  public static final double UPDATE_PERIOD = 0.02;

  public static final class PivotConstants {
    /** Maximum angle for the pivot to move to, in degrees */
    public static final double MAX_ANGLE = 120;

    /** Minimum angle for the pivot to move to, in degrees */
    public static final double MIN_ANGLE = 3;

    /** */
    public static final float VOLTAGE_LIMIT = 5;

    /** ID of the left pivot sparkmax */
    public static final int LEFT_MOTOR_ID = 10;

    /** */
    public static final boolean LEFT_MOTOR_INVERTED = true;

    /** */
    public static final int LEFT_MOTOR_CURRENT_LIMIT = 80;

    /** ID of the right pivot sparkmax */
    public static final int RIGHT_MOTOR_ID = 11;

    /** */
    public static final boolean RIGHT_MOTOR_INVERTED = false;

    /** */
    public static final int RIGHT_MOTOR_CURRENT_LIMIT = 80;

    /** */
    public static final int ENCODER_ID = 1;

    /** */
    public static final double ENCODER_POSITION_OFFSET = 26.406295410157384;

    /** */
    public static final double GEAR_RATIO = 378;

    /** */
    public static final ArmFeedforward FF_MODEL =
        new ArmFeedforward(0.11164, 0.0090459, 0.11954, 0.0090459);

    /** */
    public static final PIDController PID_CONTROLLER = new PIDController(.09361, 0, 0);

    /** */
    public static final TrapezoidProfile TRAPEZOID_PROFILE =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    /** Ramp Rate of the pivot System ID in volts per second */
    public static final double SYS_ID_RAMP_RATE = 1;

    /** Setp Voltage of the pivot System ID in volts */
    public static final double SYS_ID_STEP_VALUE = 7;

    /** Timeout of the pivot System ID in volts */
    public static final double SYS_ID_TIMEOUT = 30;

    /** How many degrees the pivot can be off its goal position for it to be sufficient */
    public static final double ALLOWED_DEVIANCE = 1.15;

    /** */
    public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);

    /** */
    public static final double DEFAULT_ANGLE = 90;

    /** */
    public static final double SIM_GOAL_POSITION = 1.05;

    /** */
    public static final double SIM_SETPOINT_POSITION = 1.05;

    /** */
    public static final SingleJointedArmSim SIM =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            300,
            0.17,
            0.500,
            Units.degreesToRadians(MIN_ANGLE),
            Units.degreesToRadians(MAX_ANGLE),
            true,
            Units.degreesToRadians(45));

    public static final int RATCHET_PIN_1_ID = 8;
    public static final int RATCHET_PIN_2_ID = 9;
  }

  public static final class ElevatorConstants {

    /** Maximum height for the elevator to move to, in meters */
    public static final double MAX_HEIGHT = 1.15;

    /** Minimum height for the elevator to move to, in meters */
    public static final double MIN_HEIGHT = 0;

    /** */
    public static final float VOLTAGE_LIMIT = 5;

    /** ID of the bottom elevator kraken */
    public static final int BOTTOM_MOTOR_ID = 12;

    /** */
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    /** */
    public static final int BOTTOM_MOTOR_CURRENT_LIMIT = 50;

    /** ID of the right pivot kraken */
    public static final int TOP_MOTOR_ID = 13;

    /** */
    public static final boolean TOP_MOTOR_INVERTED = false;

    /** */
    public static final int TOP_MOTOR_CURRENT_LIMIT = 50;

    /** */
    /** */
    public static final ElevatorFeedforward FF_MODEL = new ElevatorFeedforward(1, 1, 1, 1);

    /** */
    public static final PIDController PID_CONTROLLER = new PIDController(1, 0, 0);

    /** */
    public static final TrapezoidProfile TRAPEZOID_PROFILE =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    /** Ramp Rate of the elevator System ID in volts per second */
    public static final double SYS_ID_RAMP_RATE = 0.2;

    /** Setp Voltage of the elevator System ID in volts */
    public static final double SYS_ID_STEP_VALUE = 7;

    /** Timeout of the elevator System ID in volts */
    public static final double SYS_ID_TIMEOUT = 30;

    /** How many degrees the elevator can be off its goal position for it to be sufficient */
    public static final double ALLOWED_DEVIANCE = 1.15;

    /** */
    public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);

    /** */
    public static final double DEFAULT_HEIGHT = 90;

    /** */
    public static final double SIM_GOAL_POSITION = 1.05;

    /** */
    public static final double SIM_SETPOINT_POSITION = 1.05;

    /** */
    public static final SingleJointedArmSim SIM =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            300,
            0.17,
            0.500,
            Units.degreesToRadians(MIN_HEIGHT),
            Units.degreesToRadians(MAX_HEIGHT),
            true,
            Units.degreesToRadians(45));

    /* Absolute highest point from the base the elevator can reach in inches*/
    public static final double absoluteMaxExtension = 6;

    public static final TrapezoidProfile elevatorProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    public static final double elevatorCurrentLimit = 3;

    public static final double moveVoltage = 5.0;

    /* Device IDs */
    public static final int motorID = 12;

    public static final double rotToMetMultFactor = 1.25 / 42.5;
  }

  public static final class WristConstants {

    /** Maximum angle for the wrist to move to, in degrees */
    public static final double MAX_ANGLE = 135;

    /** Minimum angle for the wrist to move to, in degrees */
    public static final double MIN_ANGLE = -15;

    /** */
    public static final float VOLTAGE_LIMIT = 5;

    /** ID of the wrist sparkmax */
    public static final int MOTOR_ID = 14;

    /** */
    public static final boolean MOTOR_INVERTED = false;

    /** */
    public static final int MOTOR_CURRENT_LIMIT = 120;

    /** */
    public static final int ENCODER_ID = 2;

    /** */
    public static final double MOTOR_RATIO = 7;

    /** */
    public static final double ENCODER_POSITION_OFFSET = -279.59280098982003 / 2;

    /** */
    public static final double GEAR_RATIO = 6;

    /** */
    public static final ArmFeedforward FF_MODEL = new ArmFeedforward(0, 0, 0, 0);

    /** */
    public static final PIDController PID_CONTROLLER = new PIDController(0, 0, 0);

    /** */
    public static final TrapezoidProfile TRAPEZOID_PROFILE =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    /** Ramp Rate of the wrist System ID in volts per second */
    public static final double SYS_ID_RAMP_RATE = 1;

    /** Setp Voltage of the wrist System ID in volts */
    public static final double SYS_ID_STEP_VALUE = 7;

    /** Timeout of the wrist System ID in volts */
    public static final double SYS_ID_TIMEOUT = 30;

    /** How many degrees the wrist can be off its goal position for it to be sufficient */
    public static final double ALLOWED_DEVIANCE = 1.15;

    /** */
    public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);

    /** */
    public static final double DEFAULT_ANGLE = 0;

    /** */
    public static final double SIM_GOAL_POSITION = 1.05;

    /** */
    public static final double SIM_SETPOINT_POSITION = 1.05;

    /** */
    public static final SingleJointedArmSim SIM =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            300,
            0.17,
            0.500,
            Units.degreesToRadians(MIN_ANGLE),
            Units.degreesToRadians(MAX_ANGLE),
            true,
            Units.degreesToRadians(45));
  }

  public static final class IntakeConstants {

    public static final int intakeMotorID = 15;

    public static final double intakeMotorCurrentLimit = 5;
  }

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
                  Rotation2d.fromRadians(0.7915340865489908 * -1), // FL
                  Rotation2d.fromRadians((-0.23316507975861744 + Math.PI) * -1), // FR
                  Rotation2d.fromRadians(-0.09050486648525283 * -1), // BL
                  Rotation2d.fromRadians(-3.0802334220743677 * -1) // BR
                };
          };

      public static final Boolean[] turnMotorInversion =
          switch (currentRobot) {
            case DORY ->
                new Boolean[] {
                  true, true, false, true,
                };
            case NAUTILUS ->
                new Boolean[] {
                  true, true, true, true,
                };
          };

      public static final Boolean[] driveMotorInversion =
          switch (currentRobot) {
            case DORY ->
                new Boolean[] {
                  true, true, true, false,
                };
            case NAUTILUS ->
                new Boolean[] {
                  true, true, true, false,
                };
          };
    }

    public static final class reefTargetConstants {
      // Defines new variables for the x/y translations for the target positions (currently at
      // placeholders)
      // Origin to bumper ~0.4572 m
      public static final double targetLevel1X = 0.3048 + 0.4572; // TODO: Measure
      public static final double targetLevel2X = 0.3048 + 0.4572; // TODO: Measure
      public static final double targetLevel3X = 0.3048 + 0.4572; // TODO: Measure
      public static final double targetLevel4X = 0.4064 + 0.4572; // Measured but not exact

      public static final double targetY = 0.1793875;

      public static final double targetAngle = Radians.convertFrom(180, Degrees);

      // Define reef centerpoints (blue alliance)
      public static final double reefCenterX = 4.489323;
      public static final double reefCenterY = 4.0259;
    }
  }

  public static final class PathingConstants {
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

  public static final class AprilTagConstants {
    /**
     * The layout of the april tags on the field. Comps in PNW should use welded, and the
     * differences between welded and AndyMark are very small.
     */
    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static PoseStrategy poseStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

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
                  Units.inchesToMeters(-12.375),
                  Units.inchesToMeters(8),
                  Units.inchesToMeters(10.5)),
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
}
