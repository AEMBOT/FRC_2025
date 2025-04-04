package frc.robot.constants;

public class WristConstants {
  /** Maximum angle for the wrist to move to, in degrees */
  public static final double MAX_ANGLE = 135;

  /** Minimum angle for the wrist to move to, in degrees */
  public static final double MIN_ANGLE = -25;

  /** ID of the wrist sparkmax */
  public static final int MOTOR_ID = 14;

  /** */
  public static final int MOTOR_CURRENT_LIMIT = 120;

  /** */
  public static final int ENCODER_ID = 2;

  /** */
  public static final double MOTOR_RATIO = 25;

  /** */
  public static final double ENCODER_POSITION_OFFSET = 47.02818267570456 * -1;

  /** */
  public static final double GEAR_RATIO = 6;

  /** How many degrees the wrist can be off its goal position for it to be sufficient */
  public static final double ALLOWED_DEVIANCE = 6;
}
