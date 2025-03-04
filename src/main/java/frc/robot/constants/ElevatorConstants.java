package frc.robot.constants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ElevatorConstants {
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
