package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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

  public static final Robot currentRobot =
      robotJumper.get()
          ? Robot.NAUTILUS
          : Robot.DORY; // Minor todo, make this not tenery for clarity

  public static final Mode currentMode = Mode.REAL;

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
      public static final double targetX = 0.5;
      public static final double targetY = 0.75;
      public static final double targetAngle = 0;

      // Define reef centerpoints (blue alliance)
      public static final double reefCenterX = 4.489323;
      public static final double reefCenterY = 4.0259;
    }
  }

  public static final class PivotConstants {
    /** Maximum angle for the pivot to move to, in degrees */
    public static final double pivotMaxAngle = 100;

    /** Minimum angle for the pivot to move to, in degrees */
    public static final double pivotMinAngle = 30;

    /** ID of the left pivot sparkmax */
    public static final int pivotLeftMotorID = 10;

    /** */
    public static final boolean pivotLeftMotorInverted = false;

    /** */
    public static final int pivotLeftMotorCurrentLimit = 50;

    /** ID of the right pivot sparkmax */
    public static final int pivotRightMotorID = 11;

    /** */
    public static final boolean pivotRightMotorInverted = false;

    /** */
    public static final int pivotRightMotorCurrentLimit = 50;

    /** */
    public static final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(1);

    /** */
    public static final double pivotEncoderPositionOffset = -156.67488616687214;

    /** */
    public static final double gearRatio = 93.3333333;

    /** */
    public static final ArmFeedforward pivotFFModel = new ArmFeedforward(0.1, 0.1, 0.5, 0.1);

    /** */
    public static final PIDController pivotPIDController = new PIDController(2, 0, 0.00);

    /** */
    public static final TrapezoidProfile pivotProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 5));

    /** Ramp Rate of the pivot System ID in volts per second */
    public static final double pivotSysIdRampRate = 0.2;

    /** Setp Voltage of the pivot System ID in volts */
    public static final double pivotSysIdStepVolt = 7;

    /** Timeout of the pivot System ID in volts */
    public static final double pivotSysIdTimeout = 30;

    /** How many degrees the pivot can be off its goal position for it to be sufficient */
    public static final double pivotAngleAllowedDeviance = 1.15;

    /** */
    public static final Translation3d pivotTranslationFromRobot = new Translation3d(-0.2, 0, 0.255);

    /** */
    public static final double pivotDefaultAngle = 90;

    /** */
    public static final double pivotSimGoalPosition = 1.05;

    /** */
    public static final double pivotSimSetpointPosition = 1.05;

    /** */
    public static final SingleJointedArmSim pivotSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            300,
            0.17,
            0.500,
            Units.degreesToRadians(pivotMinAngle),
            Units.degreesToRadians(pivotMaxAngle),
            true,
            Units.degreesToRadians(45));
  }

  public static final class ElevatorConstants {

    /* Absolute highest point from the base the elevator can reach in inches*/
    public static final double absoluteMaxExtension = 6;

    public static final TrapezoidProfile elevatorProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1, 2));

    public static final double elevatorCurrentLimit = 3;

    public static final double moveVoltage = 5.0;

    /* Device IDs */
    public static final int motorID = 12;

    public static final double rotToInMultFactor = 1;
  }

  public static final class WristConstants {
    public static final double encoderOffset = 194.10106985252673 * -1;
    public static final double wristMaxAngle = -90;
    public static final double wristMinAngle = 90;
    public static final double deadzone = 5.0;

    /* Device IDs */
    public static final int motorID = 14;
    public static final DutyCycleEncoder wristEncoder = new DutyCycleEncoder(2);

    public static final TrapezoidProfile wristProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(2, 5));

    public static final ArmFeedforward wristFFModel = new ArmFeedforward(0.1, 0.1, 0.5, 0.1);

    public static final PIDController wristPIDController = new PIDController(2, 0, 0.00);

    public static final double wristMotorCurrentLimit = 0.25;
  }

  public static final class IntakeConstants {

    public static final int intakeMotorID = 15;

    public static final double intakeMotorCurrentLimit = 5;
  }
}
