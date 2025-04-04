package frc.robot.constants;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

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

  /** How many meters the elevator can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 0.01;

  /** */
  public static final double DEFAULT_HEIGHT = 90;

  /** */
  public static final double SIM_GOAL_POSITION = 1.05;

  /** */
  public static final double SIM_SETPOINT_POSITION = 1.05;

  /** */
  public static final ElevatorSim SIM =
      new ElevatorSim(DCMotor.getKrakenX60(2), 1, 2, 0.1, 0.01, MAX_HEIGHT, true, 0.1);

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
