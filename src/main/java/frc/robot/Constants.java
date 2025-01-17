package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;


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
  public static final Robot currentRobot = Robot.BUNNYBOT;
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
    BUNNYBOT,
    LIGHTCYCLE
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

      public static final Rotation2d[] absoluteEncoderOffset = switch (currentRobot) {
        case BUNNYBOT -> new Rotation2d[] {
          Rotation2d.fromRadians(2.291767297101148), // FL
          Rotation2d.fromRadians(2.409883817768342 + Math.PI), // FR
          Rotation2d.fromRadians(1.928213850372251), // BL
          Rotation2d.fromRadians(1.73493227109866 + Math.PI) // BR
        };
        case LIGHTCYCLE -> new Rotation2d[] { // This is not currently correct
          Rotation2d.fromRadians(2.6599226861937018), // FL
          Rotation2d.fromRadians(-2.9206994201342606 + Math.PI), // FR
          Rotation2d.fromRadians(1.064582666792635), // BL
          Rotation2d.fromRadians(-2.406815856192571 + Math.PI) // BR
        };
      };
    }
  }

  
  public static final class PivotConstants { 
    /** Maximum angle for the pivot to move to, in degrees */
    public static final double pivotMaxAngle = 170;
    /** Minimum angle for the pivot to move to, in degrees */
    public static final double pivotMinAngle = 50;
    /** ID of the left pivot sparkmax */
    public static final int pivotLeftMotorID = currentRobot == Robot.BUNNYBOT
      ? 10
      : 0; // unused on llightcycle
    /**  */
    public static final boolean pivotLeftMotorInverted = false;
    /**  */
    public static final int pivotLeftMotorCurrentLimit = 10;
    /** ID of the right pivot sparkmax */
    public static final int pivotRightMotorID = currentRobot == Robot.BUNNYBOT
      ? 11
      : 0; // unused on llightcycle
    /**  */
    public static final boolean pivotRightMotorInverted = false;
    /**  */
    public static final int pivotRightMotorCurrentLimit = 10;
    /**  */
    public static final CANcoder pivotCANcoder = new CANcoder(3, "");
    /**  */
    public static final double pivotCANcoderPositionOffset = 0.25;
    /**  */
    public static final double gearRatio = 93.3333333;
    /**  */
    public static final ArmFeedforward pivotFFModel = new ArmFeedforward(
      0.35, 
      0.35, 
      1.79, 
      0.3);
    /**  */
    public static final PIDController pivotPIDController = new PIDController(
      12, 
      0, 
      0.00);
    /**  */
    public static final TrapezoidProfile pivotProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      2,
      5));
    /** Ramp Rate of the pivot System ID in volts per second */
    public static final double pivotSysIdRampRate = 0.2;
    /** Setp Voltage of the pivot System ID in volts */
    public static final double pivotSysIdStepVolt = 7;
    /** Timeout of the pivot System ID in volts */
    public static final double pivotSysIdTimeout = 30;
    /** How many degrees the pivot can be off its goal position for it to be sufficient */
    public static final double pivotAngleAllowedDeviance = 1.15;
    /**  */
    public static final Translation3d pivotTranslationFromRobot = new Translation3d(-0.2, 0, 0.255);
    /**  */
    public static final double pivotDefaultAngle = 90;
    /**  */
    public static final double pivotSimGoalPosition = 1.05;
    /**  */
    public static final double pivotSimSetpointPosition = 1.05;
    /**  */
    public static final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      DCMotor.getNEO(2), 
      300, 
      0.17, 
      0.500, 
      Units.degreesToRadians(pivotMinAngle), 
      Units.degreesToRadians(pivotMaxAngle), 
      true, 
      Units.degreesToRadians(45));
  }
  
  public static class LoaderConstants {
    /* PORTS */
    public static final int MOTOR_PORT = currentRobot == Robot.BUNNYBOT
      ? 15
      : 0; // unused on llightcycle
    /* Voltages */
    public static final double MOTOR_VOLTAGE = 12; // PLACEHOLDER VALUE
    /* CURRENT LIMITS */
    public static final int MOTOR_CURRENT_LIMIT = 25; // PLACEHOLDER VALUE
  }
  
}