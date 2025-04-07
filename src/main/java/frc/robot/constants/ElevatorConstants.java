package frc.robot.constants;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorConstants {
  /** Maximum height for the elevator to move to, in meters */
  public static final double MAX_HEIGHT = 1.15;

  /** Minimum height for the elevator to move to, in meters */
  public static final double MIN_HEIGHT = 0;

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

  /** How many meters the elevator can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 0.01;

  public static final double rotToMetMultFactor = 1.25 / 42.5;

  /** */
  public static final double ZERO_POSITION = 0; // TODO tune

  /** */
  public static final double ZEROING_VOLTAGE = 0.5; // TODO tune

  /** */
  public static final double ELEVATOR_ZEROING_MAX_AMPS = 5; // TODO tune

  /** */
  public static final ElevatorSim SIM =
      new ElevatorSim(DCMotor.getKrakenX60(2), 1, 2, 0.1, 0.001, MAX_HEIGHT, true, 0.1);
}
