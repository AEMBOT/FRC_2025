package frc.robot.constants;

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
}
