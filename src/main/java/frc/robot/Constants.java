package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public final class Constants {
    public static final DigitalInput robotJumper = new DigitalInput(0);
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    public static final Robot currentRobot = robotJumper.get() ? Robot.NAUTILUS : Robot.DORY; // TODO: Confirm robotJumber works, we may have two jumpers on Nautilus

    public enum Mode {
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
      /**  */
      public static final float VOLTAGE_LIMIT = 5;
      /** ID of the left pivot sparkmax */
      public static final int LEFT_MOTOR_ID = 10;
      /**  */
      public static final boolean LEFT_MOTOR_INVERTED = true;
      /**  */
      public static final int LEFT_MOTOR_CURRENT_LIMIT = 80;

      /** ID of the right pivot sparkmax */
      public static final int RIGHT_MOTOR_ID = 11;
      /**  */
      public static final boolean RIGHT_MOTOR_INVERTED = false;
      /**  */
      public static final int RIGHT_MOTOR_CURRENT_LIMIT = 80;
      /**  */
      public static final int ENCODER_ID = 1;
      /**  */
      public static final double ENCODER_POSITION_OFFSET = 26.406295410157384;
      /**  */
      public static final double GEAR_RATIO = 93.3333333;
      /**  */
      public static final ArmFeedforward FF_MODEL = new ArmFeedforward(
        0.11164, 
        0.0090459, 
        0.11954, 
        0.0090459);
      /**  */
      public static final PIDController PID_CONTROLLER = new PIDController(
        .09361, 
        0, 
        0);
      /**  */
      public static final TrapezoidProfile TRAPEZOID_PROFILE = new TrapezoidProfile(new TrapezoidProfile.Constraints(
        1,
        2));
      /** Ramp Rate of the pivot System ID in volts per second */
      public static final double SYS_ID_RAMP_RATE = 1;
      /** Setp Voltage of the pivot System ID in volts */
      public static final double SYS_ID_STEP_VALUE = 7;
      /** Timeout of the pivot System ID in volts */
      public static final double SYS_ID_TIMEOUT = 30;
      /** How many degrees the pivot can be off its goal position for it to be sufficient */
      public static final double ALLOWED_DEVIANCE = 1.15;
      /**  */
      public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);
      /**  */
      public static final double DEFAULT_ANGLE = 90;
      /**  */
      public static final double SIM_GOAL_POSITION = 1.05;
      /**  */
      public static final double SIM_SETPOINT_POSITION = 1.05;
      /**  */
      public static final SingleJointedArmSim SIM = new SingleJointedArmSim(
        DCMotor.getNEO(2), 
        300, 
        0.17, 
        0.500, 
        Units.degreesToRadians(MIN_ANGLE), 
        Units.degreesToRadians(MAX_ANGLE), 
        true, 
        Units.degreesToRadians(45));
  }

    public static final class ElevatorConstants {

            /** Maximum height for the elevator to move to, in meters */
            public static final double MAX_HEIGHT = 1.15;
            /** Minimum height for the elevator to move to, in meters */
            public static final double MIN_HEIGHT = 0;
            /**  */
            public static final float VOLTAGE_LIMIT = 5;
            /** ID of the bottom elevator kraken */
            public static final int BOTTOM_MOTOR_ID = 12;
            /**  */
            public static final boolean BOTTOM_MOTOR_INVERTED = false;
            /**  */
            public static final int BOTTOM_MOTOR_CURRENT_LIMIT = 50;
      
            /** ID of the right pivot kraken */
            public static final int TOP_MOTOR_ID = 13;
            /**  */
            public static final boolean TOP_MOTOR_INVERTED = false;
            /**  */
            public static final int TOP_MOTOR_CURRENT_LIMIT = 50;
            /**  */
            /**  */
            public static final ElevatorFeedforward FF_MODEL = new ElevatorFeedforward(
              1,
              1, 
              1, 
              1);
            /**  */
            public static final PIDController PID_CONTROLLER = new PIDController(
              1, 
              0, 
              0);
            /**  */
            public static final TrapezoidProfile TRAPEZOID_PROFILE = new TrapezoidProfile(new TrapezoidProfile.Constraints(
              1,
              2));
            /** Ramp Rate of the elevator System ID in volts per second */
            public static final double SYS_ID_RAMP_RATE = 0.2;
            /** Setp Voltage of the elevator System ID in volts */
            public static final double SYS_ID_STEP_VALUE = 7;
            /** Timeout of the elevator System ID in volts */
            public static final double SYS_ID_TIMEOUT = 30;
            /** How many degrees the elevator can be off its goal position for it to be sufficient */
            public static final double ALLOWED_DEVIANCE = 1.15;
            /**  */
            public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);
            /**  */
            public static final double DEFAULT_HEIGHT = 90;
            /**  */
            public static final double SIM_GOAL_POSITION = 1.05;
            /**  */
            public static final double SIM_SETPOINT_POSITION = 1.05;
            /**  */
            public static final SingleJointedArmSim SIM = new SingleJointedArmSim(
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

      public static final TrapezoidProfile elevatorProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      1,
      2));

      public static final double elevatorCurrentLimit = 3;

      public static final double moveVoltage = 5.0;

        /* Device IDs */
      public static final int motorID = 12;

      public static final double rotToMetMultFactor = 1.25/42.5;
    }

    public static final class WristConstants {

            /** Maximum angle for the wrist to move to, in degrees */
            public static final double MAX_ANGLE = 135;
            /** Minimum angle for the wrist to move to, in degrees */
            public static final double MIN_ANGLE = -15;
            /**  */
            public static final float VOLTAGE_LIMIT = 5;
            /** ID of the wrist sparkmax */
            public static final int MOTOR_ID = 14;
            /**  */
            public static final boolean MOTOR_INVERTED = false;
            /**  */
            public static final int MOTOR_CURRENT_LIMIT = 120;
            /**  */
            public static final int ENCODER_ID = 2;
            /**  */
            public static final double MOTOR_RATIO = 7;
            /**  */
            public static final double ENCODER_POSITION_OFFSET = -279.59280098982003 / 2;
            /**  */
            public static final double GEAR_RATIO = 6;
            /**  */
            public static final ArmFeedforward FF_MODEL = new ArmFeedforward(
              0, 
              0, 
              0, 
              0);
            /**  */
            public static final PIDController PID_CONTROLLER = new PIDController(
              0, 
              0, 
              0);
            /**  */
            public static final TrapezoidProfile TRAPEZOID_PROFILE = new TrapezoidProfile(new TrapezoidProfile.Constraints(
              1,
              2));
            /** Ramp Rate of the wrist System ID in volts per second */
            public static final double SYS_ID_RAMP_RATE = 1;
            /** Setp Voltage of the wrist System ID in volts */
            public static final double SYS_ID_STEP_VALUE = 7;
            /** Timeout of the wrist System ID in volts */
            public static final double SYS_ID_TIMEOUT = 30;
            /** How many degrees the wrist can be off its goal position for it to be sufficient */
            public static final double ALLOWED_DEVIANCE = 1.15;
            /**  */
            public static final Translation3d TRANSLATION_FROM_ROBOT = new Translation3d(-0.2, 0, 0.255);
            /**  */
            public static final double DEFAULT_ANGLE = 0;
            /**  */
            public static final double SIM_GOAL_POSITION = 1.05;
            /**  */
            public static final double SIM_SETPOINT_POSITION = 1.05;
            /**  */
            public static final SingleJointedArmSim SIM = new SingleJointedArmSim(
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
}
}
package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;

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
                  Rotation2d.fromRadians(2.6599226861937018), // FL
                  Rotation2d.fromRadians(-2.9206994201342606 + Math.PI), // FR
                  Rotation2d.fromRadians(1.064582666792635), // BL
                  Rotation2d.fromRadians(-2.406815856192571 + Math.PI) // BR
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
}
